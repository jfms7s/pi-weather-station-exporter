#!/usr/bin/env python3

import asyncio
import itertools
import logging
import threading
import time
import types

from prometheus_client import REGISTRY, Gauge, start_http_server

log = logging.getLogger("weather_station")

HTTP_PORT = 8705

WIND_DIRECTION_PIN = 17
WIND_DIRECTION_STEPS = 96  # number of rose positions the sensor can report
PULSES_PER_ROTATION = 40   # anemometer edges (rising + falling) per full rotation

DHT_PIN_NAME = "D19"  # attribute of the `board` module, resolved in main()
DHT_POLL_INTERVAL_SECONDS = 2

BME280_I2C_ADDRESS_DEFAULT = None  # library default
BMP3XX_I2C_ADDRESS = 0x76
BMP_PRESSURE_OVERSAMPLING = 8
BMP_TEMPERATURE_OVERSAMPLING = 2

VEML6075_INTEGRATION_TIME_MS = 200


def load_hardware():
    """Import the hardware driver modules. Kept out of module level so the script
    can be imported (and tested) on a machine that is not a Raspberry Pi."""
    import adafruit_bmp3xx
    import adafruit_dht
    import adafruit_pcf8591.pcf8591 as PCF
    import adafruit_veml6075
    import board
    import busio
    from adafruit_bme280 import basic as adafruit_bme280
    from adafruit_pcf8591.analog_in import AnalogIn
    from RPi import GPIO

    return types.SimpleNamespace(
        board=board,
        busio=busio,
        adafruit_veml6075=adafruit_veml6075,
        adafruit_dht=adafruit_dht,
        adafruit_bmp3xx=adafruit_bmp3xx,
        adafruit_bme280=adafruit_bme280,
        GPIO=GPIO,
        PCF=PCF,
        AnalogIn=AnalogIn,
    )


class FastWriteCounter:
    """Lock-free-on-writes counter relying on itertools.count()'s atomicity under the GIL."""

    def __init__(self):
        self._number_of_read = 0
        self._counter = itertools.count()
        self._read_lock = threading.Lock()

    def increment(self):
        next(self._counter)

    def value(self):
        with self._read_lock:
            value = next(self._counter) - self._number_of_read
            self._number_of_read += 1
        return value


def safe_gauge(name, documentation, read_fn, registry=REGISTRY):
    """Register a Gauge whose value comes from read_fn, without letting a failing
    sensor read raise inside prometheus_client's collection and break the whole scrape."""

    def wrapped():
        try:
            return read_fn()
        except Exception:
            log.warning("failed to read metric %s", name, exc_info=True)
            return float("nan")

    Gauge(name, documentation, registry=registry).set_function(wrapped)


def register_uv_metrics(veml, registry=REGISTRY):
    safe_gauge("uv_sample_time", "time between samples", lambda: veml.integration_time,
               registry=registry)
    safe_gauge("uva_count", "uva count between samples", lambda: veml.uva,
               registry=registry)
    safe_gauge("uvb_count", "uvb count between samples", lambda: veml.uvb,
               registry=registry)
    safe_gauge("uv_index", "Index UV", lambda: veml.uv_index,
               registry=registry)


def build_pressure_sensors(hw, i2c):
    bme280 = hw.adafruit_bme280.Adafruit_BME280_I2C(i2c)
    bmp = hw.adafruit_bmp3xx.BMP3XX_I2C(i2c, BMP3XX_I2C_ADDRESS)

    bmp.pressure_oversampling = BMP_PRESSURE_OVERSAMPLING
    bmp.temperature_oversampling = BMP_TEMPERATURE_OVERSAMPLING
    return bme280, bmp


def register_pressure_metrics(bme280, bmp, registry=REGISTRY):
    safe_gauge("temperature_bme280", "temperature in Centigrade", lambda: bme280.temperature,
               registry=registry)
    safe_gauge("pressure_bme280", "pressure", lambda: bme280.pressure,
               registry=registry)
    safe_gauge("humidity_bme280", "humidity", lambda: bme280.humidity,
               registry=registry)

    safe_gauge("temperature_3xx", "temperature in Centigrade", lambda: bmp.temperature,
               registry=registry)
    safe_gauge("pressure_3xx", "pressure", lambda: bmp.pressure,
               registry=registry)


def setup_anemometer(gpio, counter):
    gpio.setup(WIND_DIRECTION_PIN, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.add_event_detect(WIND_DIRECTION_PIN, gpio.BOTH, callback=lambda _: counter.increment())


def register_wind_metrics(vane, counter, registry=REGISTRY):
    safe_gauge("rose_direction", "current rose direction",
               lambda: (vane.value / 65535) * WIND_DIRECTION_STEPS,
               registry=registry)
    safe_gauge("rose_power_percentage", "current rose power_percentage",
               lambda: vane.value / 65535,
               registry=registry)
    safe_gauge("rose_power", "current rose power",
               lambda: (vane.value / 65535) * vane.reference_voltage,
               registry=registry)
    safe_gauge("anemometer_rotations", "number of complete rotations since start",
               lambda: counter.value() / PULSES_PER_ROTATION,
               registry=registry)
    safe_gauge("anemometer_activation", "number of signals sent by anemometer",
               lambda: counter.value(),
               registry=registry)


def scrape_dht_metrics(adafruit_dht, dht_pin, registry=REGISTRY):
    dht_device = adafruit_dht.DHT22(dht_pin)
    temperature_metric = Gauge("temperature", "temperature in Centigrade", registry=registry)
    humidity_metric = Gauge("humidity", "humidity percentage", registry=registry)

    while True:
        try:
            temperature = dht_device.temperature
            humidity = dht_device.humidity
        except RuntimeError as error:
            log.warning("DHT read failed: %s", error)
            time.sleep(DHT_POLL_INTERVAL_SECONDS)
            continue
        except Exception:
            log.exception("unexpected error reading DHT sensor")
            time.sleep(DHT_POLL_INTERVAL_SECONDS)
            continue

        if humidity is not None and temperature is not None:
            temperature_metric.set(temperature)
            humidity_metric.set(humidity)
            log.info("Temp=%.1f*C Humidity=%.1f%%", temperature, humidity)

        time.sleep(DHT_POLL_INTERVAL_SECONDS)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    hw = load_hardware()
    gpio = hw.GPIO

    start_http_server(HTTP_PORT)
    log.info("metrics server listening on :%d", HTTP_PORT)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    i2c = hw.busio.I2C(hw.board.SCL, hw.board.SDA)
    veml = hw.adafruit_veml6075.VEML6075(i2c, integration_time=VEML6075_INTEGRATION_TIME_MS)
    pcf = hw.PCF.PCF8591(i2c)

    gpio.cleanup()
    gpio.setmode(gpio.BCM)

    try:
        register_uv_metrics(veml)

        bme280, bmp = build_pressure_sensors(hw, i2c)
        register_pressure_metrics(bme280, bmp)

        vane = hw.AnalogIn(pcf, hw.PCF.A0)
        counter = FastWriteCounter()
        setup_anemometer(gpio, counter)
        register_wind_metrics(vane, counter)

        dht_pin = getattr(hw.board, DHT_PIN_NAME)
        loop.run_in_executor(None, scrape_dht_metrics, hw.adafruit_dht, dht_pin)
        loop.run_forever()
    finally:
        gpio.cleanup()


if __name__ == "__main__":
    main()
