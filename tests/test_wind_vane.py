import hardware_stubs
import pytest
from prometheus_client import CollectorRegistry

import Weather_Station as ws

FULL_SCALE_8BIT = 255 << 8  # the largest AnalogIn.value the PCF8591 driver returns


def test_vane_fraction_bounds():
    assert ws.vane_fraction(0) == 0.0
    assert ws.vane_fraction(128 << 8) == pytest.approx((128 << 8) / ws.WIND_VANE_FULL_SCALE)


def test_vane_position_and_volts_scale_the_fraction():
    assert ws.vane_position(1.0) == 96.0
    assert ws.vane_position(0.5) == 48.0
    assert ws.vane_volts(1.0, 3.3) == 3.3
    assert ws.vane_volts(0.0, 3.3) == 0.0


@pytest.mark.parametrize("raw", [0, 1 << 8, 128 << 8, FULL_SCALE_8BIT])
def test_vane_gauges_keep_the_previous_values(monkeypatch, raw):
    # The helpers must export exactly what the three inline formulas did (value / 65535).
    monkeypatch.setattr(hardware_stubs.AnalogIn, "readings",
                        {"value": raw, "reference_voltage": 3.3})
    vane = hardware_stubs.AnalogIn()
    registry = CollectorRegistry()
    ws.start_sensor("wind_vane", lambda: vane, ws.WIND_VANE_GAUGES, registry=registry)

    assert registry.get_sample_value("rose_power_percentage") == raw / 65535
    assert registry.get_sample_value("rose_direction") == (raw / 65535) * 96
    assert registry.get_sample_value("rose_power") == (raw / 65535) * 3.3
