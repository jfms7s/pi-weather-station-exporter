import math
import subprocess
import sys

import hardware_stubs
import pytest
from conftest import REPO_ROOT
from prometheus_client import CollectorRegistry

import Weather_Station as ws


def sample(registry, name):
    value = registry.get_sample_value(name)
    assert value is not None, f"{name} not exposed"
    return value


def test_module_imports_without_hardware_modules():
    code = (
        "import sys, Weather_Station\n"
        "hw = {'board', 'busio', 'RPi', 'RPi.GPIO', 'adafruit_dht', 'adafruit_veml6075',"
        " 'adafruit_bmp3xx', 'adafruit_bme280', 'adafruit_pcf8591'}\n"
        "loaded = sorted(hw & set(sys.modules))\n"
        "assert not loaded, loaded\n"
    )
    result = subprocess.run(
        [sys.executable, "-c", code], cwd=REPO_ROOT, capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr


def test_safe_gauge_failing_read_is_nan_and_sibling_keeps_value():
    registry = CollectorRegistry()

    def broken():
        raise hardware_stubs.SensorReadError("sensor gone")

    ws.safe_gauge("broken_metric", "always fails", broken, registry=registry)
    ws.safe_gauge("healthy_metric", "always works", lambda: 42.0, registry=registry)

    assert math.isnan(sample(registry, "broken_metric"))
    assert sample(registry, "healthy_metric") == 42.0


@pytest.fixture
def hw(monkeypatch):
    hardware_stubs.install(monkeypatch)
    return ws.load_hardware()


def test_registers_uv_pressure_and_wind_metrics_with_fake_values(hw):
    registry = CollectorRegistry()
    i2c = hw.busio.I2C(hw.board.SCL, hw.board.SDA)
    veml = hw.adafruit_veml6075.VEML6075(i2c, integration_time=ws.VEML6075_INTEGRATION_TIME_MS)
    pcf = hw.PCF.PCF8591(i2c)

    ws.register_uv_metrics(veml, registry=registry)
    bme280, bmp = ws.build_pressure_sensors(hw, i2c)
    ws.register_pressure_metrics(bme280, bmp, registry=registry)
    vane = hw.AnalogIn(pcf, hw.PCF.A0)
    counter = ws.FastWriteCounter()
    ws.setup_anemometer(hw.GPIO, counter)
    ws.register_wind_metrics(vane, counter, registry=registry)

    for _ in range(80):
        hw.GPIO.callbacks[ws.WIND_DIRECTION_PIN](ws.WIND_DIRECTION_PIN)

    expected = {
        "uv_sample_time": 200,
        "uva_count": 11.0,
        "uvb_count": 12.0,
        "uv_index": 1.5,
        "temperature_bme280": 20.0,
        "pressure_bme280": 1012.0,
        "humidity_bme280": 55.0,
        "temperature_3xx": 19.0,
        "pressure_3xx": 1013.0,
        "rose_direction": 32768 / 65535 * ws.WIND_DIRECTION_STEPS,
        "rose_power_percentage": 32768 / 65535,
        "rose_power": 32768 / 65535 * 3.3,
    }
    for name, value in expected.items():
        assert sample(registry, name) == pytest.approx(value), name
    # FastWriteCounter.value() reports pulses since the previous read.
    assert sample(registry, "anemometer_rotations") == pytest.approx(80 / ws.PULSES_PER_ROTATION)
    assert sample(registry, "anemometer_activation") >= 0

    names = {metric.name for metric in registry.collect()}
    assert names == set(expected) | {"anemometer_rotations", "anemometer_activation"}
    assert len(names) == 14

    assert ("BMP3XX_I2C.set", "pressure_oversampling", ws.BMP_PRESSURE_OVERSAMPLING) in (
        hardware_stubs.EVENTS
    )
    assert ("setup", ws.WIND_DIRECTION_PIN, "IN", "PUD_UP") in hardware_stubs.EVENTS
    assert ("add_event_detect", ws.WIND_DIRECTION_PIN, "BOTH") in hardware_stubs.EVENTS


def test_dht_pin_name_resolves_on_board(hw):
    assert getattr(hw.board, ws.DHT_PIN_NAME) == "D19"
