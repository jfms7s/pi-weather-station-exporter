"""A sensor (or the I2C bus) failing at start-up takes down only its own gauges."""

import logging
import math

import hardware_stubs
import pytest
from prometheus_client import CollectorRegistry, generate_latest

import Weather_Station as ws

UNIT_GAUGES = {
    "veml6075": ws.UV_GAUGES,
    "bme280": ws.BME280_GAUGES,
    "bmp3xx": ws.BMP3XX_GAUGES,
    "wind_vane": ws.WIND_VANE_GAUGES,
    "anemometer": ws.ANEMOMETER_GAUGES,
}
ALL_NAMES = {name for gauges in UNIT_GAUGES.values() for name, _, _ in gauges}
I2C_UNITS = ("veml6075", "bme280", "bmp3xx", "wind_vane")


@pytest.fixture
def hw(monkeypatch):
    hardware_stubs.install(monkeypatch)
    return ws.load_hardware()


def break_unit(monkeypatch, hw, unit):
    error = ValueError(f"No I2C device for {unit}")
    if unit == "anemometer":
        hw.GPIO.raise_on_add_event_detect = RuntimeError("Failed to add edge detection")
        return
    fake = {
        "veml6075": hardware_stubs.VEML6075,
        "bme280": hardware_stubs.Adafruit_BME280_I2C,
        "bmp3xx": hardware_stubs.BMP3XX_I2C,
        "wind_vane": hardware_stubs.PCF8591,
    }[unit]
    monkeypatch.setattr(fake, "raise_on_init", error)


def start(hw, caplog):
    registry = CollectorRegistry()
    with caplog.at_level(logging.DEBUG, logger="weather_station"):
        ws.start_sensors(hw, hw.GPIO, registry=registry)
    return registry


def exposed_names(registry):
    text = generate_latest(registry).decode()
    return {line.split()[2] for line in text.splitlines() if line.startswith("# TYPE ")}


def unit_values(registry, unit):
    return [registry.get_sample_value(name) for name, _, _ in UNIT_GAUGES[unit]]


@pytest.mark.parametrize("unit", list(UNIT_GAUGES))
def test_startup_failure_of_one_unit_only_nans_its_gauges(monkeypatch, hw, caplog, unit):
    break_unit(monkeypatch, hw, unit)
    registry = start(hw, caplog)

    assert exposed_names(registry) == ALL_NAMES
    assert all(math.isnan(value) for value in unit_values(registry, unit))
    for other in UNIT_GAUGES:
        if other != unit:
            values = unit_values(registry, other)
            assert not any(math.isnan(value) for value in values), (other, values)

    errors = [r for r in caplog.records if r.levelno >= logging.ERROR]
    assert len(errors) == 1
    assert unit in errors[0].getMessage()


def test_startup_with_i2c_bus_unavailable_keeps_the_anemometer(monkeypatch, hw, caplog):
    monkeypatch.setattr(hardware_stubs.I2C, "raise_on_init", OSError("no /dev/i2c-1"))
    registry = start(hw, caplog)

    assert exposed_names(registry) == ALL_NAMES
    for unit in I2C_UNITS:
        assert all(math.isnan(value) for value in unit_values(registry, unit)), unit
    hw.GPIO.callbacks[ws.ANEMOMETER_PIN](ws.ANEMOMETER_PIN)
    assert not any(math.isnan(value) for value in unit_values(registry, "anemometer"))


@pytest.mark.parametrize("unit", list(UNIT_GAUGES))
def test_scrape_after_unavailable_startup_logs_nothing(monkeypatch, hw, caplog, unit):
    break_unit(monkeypatch, hw, unit)
    registry = start(hw, caplog)
    caplog.clear()

    with caplog.at_level(logging.DEBUG, logger="weather_station"):
        generate_latest(registry)
        generate_latest(registry)

    assert caplog.records == []
