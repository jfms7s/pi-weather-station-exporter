#!/usr/bin/env python3

import asyncio
import itertools
import logging
import threading
import time

from prometheus_client import start_http_server, Gauge

import board
import busio
import adafruit_veml6075
import adafruit_dht
import adafruit_bmp3xx
from adafruit_bme280 import basic as adafruit_bme280
import RPi.GPIO as GPIO
import adafruit_pcf8591.pcf8591 as PCF
from adafruit_pcf8591.analog_in import AnalogIn

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("weather_station")

HTTP_PORT = 8705

WIND_DIRECTION_PIN = 17
WIND_DIRECTION_STEPS = 96  # number of rose positions the sensor can report
PULSES_PER_ROTATION = 40   # anemometer edges (rising + falling) per full rotation

DHT_PIN = board.D19
DHT_POLL_INTERVAL_SECONDS = 2

BME280_I2C_ADDRESS_DEFAULT = None  # library default
BMP3XX_I2C_ADDRESS = 0x76
BMP_PRESSURE_OVERSAMPLING = 8
BMP_TEMPERATURE_OVERSAMPLING = 2

VEML6075_INTEGRATION_TIME_MS = 200


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


def safe_gauge(name, documentation, read_fn):
    """Register a Gauge whose value comes from read_fn, without letting a failing
    sensor read raise inside prometheus_client's collection and break the whole scrape."""

    def wrapped():
        try:
            return read_fn()
        except Exception:
            log.warning("failed to read metric %s", name, exc_info=True)
            return float("nan")

    Gauge(name, documentation).set_function(wrapped)


def register_uv_metrics(veml):
    safe_gauge("uv_sample_time", "time between samples", lambda: veml.integration_time)
    safe_gauge("uva_count", "uva count between samples", lambda: veml.uva)
    safe_gauge("uvb_count", "uvb count between samples", lambda: veml.uvb)
    safe_gauge("uv_index", "Index UV", lambda: veml.uv_index)


def register_pressure_metrics(i2c):
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)
    bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c, BMP3XX_I2C_ADDRESS)

    bmp.pressure_oversampling = BMP_PRESSURE_OVERSAMPLING
    bmp.temperature_oversampling = BMP_TEMPERATURE_OVERSAMPLING

    safe_gauge("temperature_bme280", "temperature in Centigrade", lambda: bme280.temperature)
    safe_gauge("pressure_bme280", "pressure", lambda: bme280.pressure)
    safe_gauge("humidity_bme280", "humidity", lambda: bme280.humidity)

    safe_gauge("temperature_3xx", "temperature in Centigrade", lambda: bmp.temperature)
    safe_gauge("pressure_3xx", "pressure", lambda: bmp.pressure)


def register_wind_metrics(pcf):
    pcf_in_0 = AnalogIn(pcf, PCF.A0)
    counter = FastWriteCounter()

    GPIO.setup(WIND_DIRECTION_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(WIND_DIRECTION_PIN, GPIO.BOTH, callback=lambda _: counter.increment())

    safe_gauge("rose_direction", "current rose direction",
               lambda: (pcf_in_0.value / 65535) * WIND_DIRECTION_STEPS)
    safe_gauge("rose_power_percentage", "current rose power_percentage",
               lambda: pcf_in_0.value / 65535)
    safe_gauge("rose_power", "current rose power",
               lambda: (pcf_in_0.value / 65535) * pcf_in_0.reference_voltage)
    safe_gauge("anemometer_rotations", "number of complete rotations since start",
               lambda: counter.value() / PULSES_PER_ROTATION)
    safe_gauge("anemometer_activation", "number of signals sent by anemometer",
               lambda: counter.value())


def scrape_dht_metrics(dht_pin):
    dht_device = adafruit_dht.DHT22(dht_pin)
    temperature_metric = Gauge("temperature", "temperature in Centigrade")
    humidity_metric = Gauge("humidity", "humidity percentage")

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
    start_http_server(HTTP_PORT)
    log.info("metrics server listening on :%d", HTTP_PORT)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    i2c = busio.I2C(board.SCL, board.SDA)
    veml = adafruit_veml6075.VEML6075(i2c, integration_time=VEML6075_INTEGRATION_TIME_MS)
    pcf = PCF.PCF8591(i2c)

    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)

    try:
        register_uv_metrics(veml)
        register_pressure_metrics(i2c)
        register_wind_metrics(pcf)

        loop.run_in_executor(None, scrape_dht_metrics, DHT_PIN)
        loop.run_forever()
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
