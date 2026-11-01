#!/usr/bin/env python3

import functools
import itertools
import logging
import os
import signal
import threading
import time
import types
from pathlib import Path

from prometheus_client import REGISTRY, Gauge, start_http_server

log = logging.getLogger("weather_station")

# Where /metrics listens. Override with the WEATHER_BIND and WEATHER_PORT environment
# variables (for example `Environment=` in a systemd unit); unset means these defaults.
HTTP_BIND = "0.0.0.0"
HTTP_PORT = 8705

# Hardware wiring: README "Hardware wiring" lists these; keep the two in step.

# Anemometer: pulse input on a GPIO pin (BCM numbering), internal pull-up, both edges
# counted.
ANEMOMETER_PIN = 17
PULSES_PER_ROTATION = 40   # anemometer edges (rising + falling) per full rotation

# Wind vane: analog voltage on a PCF8591 ADC channel (name of the channel constant in the
# adafruit_pcf8591.pcf8591 module).
WIND_VANE_ADC_CHANNEL = "A0"
# rose_direction is the vane reading on a linear, uncalibrated 0-96 scale, not a rose
# position the sensor reports.
WIND_DIRECTION_STEPS = 96
# The divisor the rose_* gauges use. adafruit_pcf8591's AnalogIn.value is the 8-bit
# reading shifted left by 8 (analog_in.py:70 in 1.0.25), so it tops out at 255 << 8 =
# 65280 and a full-scale reading gives 0.996, not 1.0. Kept at 65535 so the exported
# values stay what they were until the owner decides on 65280 (issue note
# review-wind-vane-scale-and-duplicated-conversion).
WIND_VANE_FULL_SCALE = 65535

# DHT22: read through the kernel's `dht11` IIO driver (it handles the DHT22 too),
# enabled on the Pi with `dtoverlay=dht11,gpiopin=19` (README: DHT22). The kernel does the
# bit timing and exposes the reading in sysfs, in milli-degrees Celsius and milli-percent.
IIO_DEVICES_DIR = Path("/sys/bus/iio/devices")
DHT_IIO_DRIVER_NAME = "dht11"  # the device's `name` is this, or `dht11@<pin in hex>`
DHT_POLL_INTERVAL_SECONDS = 2
# After this long without a good read the DHT gauges report NaN instead of the last value.
DHT_STALE_AFTER_SECONDS = 5 * DHT_POLL_INTERVAL_SECONDS

# I2C sensors (bus 1, board.SCL/board.SDA). VEML6075, PCF8591 and BME280 use their
# drivers' default addresses.
BME280_I2C_ADDRESS_DEFAULT = None  # library default
BMP3XX_I2C_ADDRESS = 0x76
BMP_PRESSURE_OVERSAMPLING = 8
BMP_TEMPERATURE_OVERSAMPLING = 2

VEML6075_INTEGRATION_TIME_MS = 200


def load_http_config(environ):
    """Return (bind address, port) for the metrics server from WEATHER_BIND and
    WEATHER_PORT in `environ`, defaulting to HTTP_BIND and HTTP_PORT. Port 0 picks a free
    ephemeral port. An invalid port stops start-up with a message naming the variable."""
    addr = environ.get("WEATHER_BIND", "").strip() or HTTP_BIND
    raw_port = environ.get("WEATHER_PORT", "").strip()
    if not raw_port:
        return addr, HTTP_PORT
    try:
        port = int(raw_port)
    except ValueError:
        port = None
    if port is None or not 0 <= port <= 65535:
        raise SystemExit(f"WEATHER_PORT must be an integer from 0 to 65535, got {raw_port!r}")
    return addr, port


def load_hardware():
    """Import the hardware driver modules. Kept out of module level so the script
    can be imported (and tested) on a machine that is not a Raspberry Pi."""
    import adafruit_bmp3xx
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
    sensor read raise inside prometheus_client's collection and break the whole scrape.

    A read that keeps failing logs one WARNING (with traceback) when it starts failing and
    one INFO line when it recovers, however many scrapes hit it in between: scrapes run on
    request threads, so per-request logging would let any client flood the journal. A
    sensor that flaps between failing and working still logs on every transition; moving
    reads out of the scrape (a sampler thread) is what bounds that.

    State changes are logged while holding the lock, so the log order always matches the
    order of the transitions (the last line says the state the gauge is in)."""
    failing = False
    state_lock = threading.Lock()

    def wrapped():
        nonlocal failing
        try:
            value = read_fn()
        except Exception:
            with state_lock:
                if not failing:
                    failing = True
                    log.warning("metric %s read failed; reporting NaN until it recovers",
                                name, exc_info=True)
                    return float("nan")
            log.debug("metric %s read still failing", name)
            return float("nan")
        with state_lock:
            if failing:
                failing = False
                log.info("metric %s recovered", name)
        return value

    Gauge(name, documentation, registry=registry).set_function(wrapped)


# Each physical device is one unit of failure: its gauges, as (name, HELP, read(device)),
# are registered against the live device, or as NaN when the device can't be built.
UV_GAUGES = (
    ("uv_sample_time", "time between samples", lambda veml: veml.integration_time),
    ("uva_count", "uva count between samples", lambda veml: veml.uva),
    ("uvb_count", "uvb count between samples", lambda veml: veml.uvb),
    ("uv_index", "Index UV", lambda veml: veml.uv_index),
)

BME280_GAUGES = (
    ("temperature_bme280", "temperature in Centigrade", lambda bme280: bme280.temperature),
    ("pressure_bme280", "pressure", lambda bme280: bme280.pressure),
    ("humidity_bme280", "humidity", lambda bme280: bme280.humidity),
)

BMP3XX_GAUGES = (
    ("temperature_3xx", "temperature in Centigrade", lambda bmp: bmp.temperature),
    ("pressure_3xx", "pressure", lambda bmp: bmp.pressure),
)

def vane_fraction(raw16):
    """The vane's ADC reading (AnalogIn.value, 16-bit scaled) as a fraction of
    WIND_VANE_FULL_SCALE."""
    return raw16 / WIND_VANE_FULL_SCALE


def vane_position(fraction):
    """Linear, uncalibrated position on a 0-WIND_DIRECTION_STEPS scale. A calibration
    table lookup can replace this later without touching the other conversions."""
    return fraction * WIND_DIRECTION_STEPS


def vane_volts(fraction, reference_voltage):
    return fraction * reference_voltage


WIND_VANE_GAUGES = (
    ("rose_direction", "Wind vane position on a linear 0-96 scale (uncalibrated)",
     lambda vane: vane_position(vane_fraction(vane.value))),
    ("rose_power_percentage", "Wind vane ADC reading as a fraction of full scale (0-1)",
     lambda vane: vane_fraction(vane.value)),
    ("rose_power", "Wind vane ADC reading in volts",
     lambda vane: vane_volts(vane_fraction(vane.value), vane.reference_voltage)),
)

ANEMOMETER_GAUGES = (
    ("anemometer_rotations", "number of complete rotations since start",
     lambda counter: counter.value() / PULSES_PER_ROTATION),
    ("anemometer_activation", "number of signals sent by anemometer",
     lambda counter: counter.value()),
)


def _report_nan():
    return float("nan")


def start_sensor(name, build, gauges, registry=REGISTRY):
    """Build one device with `build()` and register its gauges.

    If building it fails (missing device, wrong address, bus error, GPIO refusing edge
    detection), log one ERROR naming the unit and register its gauges as a constant NaN,
    which never raises or logs on a scrape, so every other sensor keeps working. Returns
    the device, or None when it is unavailable. There is no retry: a reseated sensor comes
    back on the next restart."""
    try:
        device = build()
    except Exception:
        log.exception("sensor %s unavailable at start-up; its metrics report NaN", name)
        for metric, documentation, _ in gauges:
            Gauge(metric, documentation, registry=registry).set_function(_report_nan)
        return None
    for metric, documentation, read in gauges:
        safe_gauge(metric, documentation, functools.partial(read, device), registry=registry)
    return device


def build_i2c(hw):
    """Open the I2C bus, or return None (logged once) if it can't be opened."""
    try:
        return hw.busio.I2C(hw.board.SCL, hw.board.SDA)
    except Exception:
        log.exception("I2C bus unavailable at start-up; every I2C sensor reports NaN")
        return None


def _require_i2c(i2c):
    if i2c is None:
        raise RuntimeError("I2C bus unavailable")
    return i2c


def build_veml6075(hw, i2c):
    return hw.adafruit_veml6075.VEML6075(
        _require_i2c(i2c), integration_time=VEML6075_INTEGRATION_TIME_MS)


def build_bme280(hw, i2c):
    return hw.adafruit_bme280.Adafruit_BME280_I2C(_require_i2c(i2c))


def build_bmp3xx(hw, i2c):
    bmp = hw.adafruit_bmp3xx.BMP3XX_I2C(_require_i2c(i2c), BMP3XX_I2C_ADDRESS)
    bmp.pressure_oversampling = BMP_PRESSURE_OVERSAMPLING
    bmp.temperature_oversampling = BMP_TEMPERATURE_OVERSAMPLING
    return bmp


def build_wind_vane(hw, i2c):
    pcf = hw.PCF.PCF8591(_require_i2c(i2c))
    return hw.AnalogIn(pcf, getattr(hw.PCF, WIND_VANE_ADC_CHANNEL))


def setup_anemometer(gpio, counter):
    gpio.setup(ANEMOMETER_PIN, gpio.IN, pull_up_down=gpio.PUD_UP)
    gpio.add_event_detect(ANEMOMETER_PIN, gpio.BOTH, callback=lambda _: counter.increment())
    return counter


def start_sensors(hw, gpio, registry=REGISTRY):
    """Build every GPIO and I2C sensor and register its gauges; one failing at start-up
    only takes down its own gauges. `gpio.setmode` must already have been called.
    Returns {unit name: device or None}."""
    i2c = build_i2c(hw)
    units = (
        ("veml6075", lambda: build_veml6075(hw, i2c), UV_GAUGES),
        ("bme280", lambda: build_bme280(hw, i2c), BME280_GAUGES),
        ("bmp3xx", lambda: build_bmp3xx(hw, i2c), BMP3XX_GAUGES),
        ("wind_vane", lambda: build_wind_vane(hw, i2c), WIND_VANE_GAUGES),
        ("anemometer", lambda: setup_anemometer(gpio, FastWriteCounter()), ANEMOMETER_GAUGES),
    )
    return {name: start_sensor(name, build, gauges, registry=registry)
            for name, build, gauges in units}


def find_dht_iio_device(devices_dir=IIO_DEVICES_DIR):
    """Return the sysfs directory of the kernel `dht11` IIO device.

    Found by driver name rather than a fixed `iio:deviceN` path, because N depends on what
    other IIO devices the Pi has and the order they probe in."""
    for device in sorted(devices_dir.glob("iio:device*")):
        try:
            name = (device / "name").read_text().strip()
        except OSError:
            continue
        if name == DHT_IIO_DRIVER_NAME or name.startswith(DHT_IIO_DRIVER_NAME + "@"):
            return device
    raise FileNotFoundError(
        f"no {DHT_IIO_DRIVER_NAME!r} IIO device under {devices_dir}; "
        "is dtoverlay=dht11,gpiopin=19 in /boot/firmware/config.txt (README: DHT22)?"
    )


def read_dht_sysfs(device_dir):
    """Read (temperature in Celsius, relative humidity in percent) from the dht11 IIO device.

    Raises OSError when the driver fails the read (it returns EIO or ETIMEDOUT)."""
    temperature = int((device_dir / "in_temp_input").read_text()) / 1000
    humidity = int((device_dir / "in_humidityrelative_input").read_text()) / 1000
    return temperature, humidity


def read_dht(devices_dir=IIO_DEVICES_DIR):
    return read_dht_sysfs(find_dht_iio_device(devices_dir))


class DhtState:
    """The DHT22's latest good reading, written by the poll thread and read by scrapes."""

    def __init__(self):
        self.lock = threading.Lock()
        self.temperature = None
        self.humidity = None
        self.last_success_monotonic = None
        self.last_success_unix = None


def poll_dht_once(read_fn, state, monotonic=time.monotonic, wall=time.time):
    """Read the DHT22 once and record the reading if both values are present.

    Exceptions from read_fn propagate to the caller (the poll loop logs them)."""
    temperature, humidity = read_fn()
    if temperature is None or humidity is None:
        return False
    with state.lock:
        state.temperature = temperature
        state.humidity = humidity
        state.last_success_monotonic = monotonic()
        state.last_success_unix = wall()
    return True


def register_dht_metrics(state, registry=REGISTRY, monotonic=time.monotonic):
    """Export the DHT22 reading as NaN until the first good read and whenever the last good
    read is older than DHT_STALE_AFTER_SECONDS. Staleness is judged at collection time, so a
    poll thread stuck inside a read still turns the gauges to NaN.

    Each gauge takes its own snapshot, so within one scrape temperature, humidity and the
    timestamp can come from adjacent polls; each value on its own is always consistent."""

    def fresh(attribute):
        def read():
            with state.lock:
                last = state.last_success_monotonic
                value = getattr(state, attribute)
            if last is None or monotonic() - last > DHT_STALE_AFTER_SECONDS:
                return float("nan")
            return value

        return read

    def last_success_unix():
        with state.lock:
            last = state.last_success_unix
        return float("nan") if last is None else last

    Gauge("temperature", "temperature in Centigrade",
          registry=registry).set_function(fresh("temperature"))
    Gauge("humidity", "humidity percentage", registry=registry).set_function(fresh("humidity"))
    Gauge("weather_sensor_last_success_timestamp_seconds",
          "Unix time of the sensor's last successful read", ["sensor"],
          registry=registry).labels(sensor="dht22").set_function(last_success_unix)


def start_dht_poller(stop, registry=REGISTRY, read_fn=read_dht):
    """Register the DHT22 gauges and start their poll loop on a daemon thread that ends
    when `stop` is set. Both get the same DhtState, so what the loop reads is what scrapes
    see. Returns (state, thread)."""
    state = DhtState()
    register_dht_metrics(state, registry=registry)
    thread = threading.Thread(target=scrape_dht_metrics, args=(state, read_fn, stop),
                              name="dht", daemon=True)
    thread.start()
    return state, thread


def scrape_dht_metrics(state, read_fn, stop):
    while not stop.is_set():
        try:
            if poll_dht_once(read_fn, state):
                log.info("Temp=%.1f*C Humidity=%.1f%%", state.temperature, state.humidity)
        except OSError as error:
            log.warning("DHT read failed: %s", error)
        except Exception:
            log.exception("unexpected error reading DHT sensor")
        stop.wait(DHT_POLL_INTERVAL_SECONDS)


# How long shutdown waits for the DHT thread to finish a read in progress; it is a daemon
# thread, so the process exits even if a read is stuck.
SHUTDOWN_JOIN_SECONDS = 2


class StopRequest:
    """Set by the SIGTERM/SIGINT handler. A plain attribute, not a threading.Event: the
    handler runs on the main thread between bytecodes, and Event.set() there can deadlock
    if the main thread was interrupted while holding the Event's internal lock."""

    def __init__(self):
        self.requested = False


# How often main() checks for a stop request; also the worst-case delay before shutdown.
STOP_POLL_SECONDS = 0.5


def install_stop_handlers(request):
    """Make SIGTERM (what `systemctl stop` sends) and SIGINT (Ctrl-C) mark `request`, so
    main() leaves its wait and runs the cleanup instead of being killed or raising mid-call."""

    def request_stop(signum, _frame):
        request.requested = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    addr, port = load_http_config(os.environ)
    hw = load_hardware()
    gpio = hw.GPIO

    start_http_server(port, addr=addr)
    log.info("metrics server listening on %s:%d", addr, port)

    gpio.cleanup()
    gpio.setmode(gpio.BCM)

    # Installed only now, right before the try whose finally does the cleanup: a signal
    # during the steps above keeps its default action (exit) instead of being deferred.
    request = StopRequest()
    install_stop_handlers(request)
    stop = threading.Event()
    dht_thread = None
    try:
        start_sensors(hw, gpio)

        _, dht_thread = start_dht_poller(stop)
        while not request.requested:
            time.sleep(STOP_POLL_SECONDS)
        log.info("stop requested, shutting down")
    finally:
        stop.set()
        if dht_thread is not None:
            dht_thread.join(SHUTDOWN_JOIN_SECONDS)
        gpio.cleanup()
        log.info("stopped")


if __name__ == "__main__":
    main()
