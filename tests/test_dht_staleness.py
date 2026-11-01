import math

import pytest
from prometheus_client import CollectorRegistry, generate_latest

import Weather_Station as ws

TIMESTAMP = "weather_sensor_last_success_timestamp_seconds"


class Clock:
    def __init__(self, start):
        self.now = start

    def __call__(self):
        return self.now


@pytest.fixture
def dht():
    """A DHT state registered into a fresh registry, with injected clocks."""
    monotonic = Clock(1000.0)
    wall = Clock(1_700_000_000.0)
    state = ws.DhtState()
    registry = CollectorRegistry()
    ws.register_dht_metrics(state, registry=registry, monotonic=monotonic)

    def poll(read_fn):
        ws.poll_dht_once(read_fn, state, monotonic=monotonic, wall=wall)

    def value(name, **labels):
        return registry.get_sample_value(name, labels or None)

    return state, registry, monotonic, wall, poll, value


def failing_read():
    raise OSError(5, "Input/output error")


def test_failing_sensor_exports_nan_not_zero(dht):
    _, registry, _, _, poll, value = dht
    for _ in range(3):
        with pytest.raises(OSError):
            poll(failing_read)

    assert math.isnan(value("temperature"))
    assert math.isnan(value("humidity"))
    assert math.isnan(value(TIMESTAMP, sensor="dht22"))
    exposition = generate_latest(registry).decode()
    assert "temperature NaN" in exposition
    assert "humidity NaN" in exposition


def test_good_read_is_exported_then_goes_stale(dht):
    _, _, monotonic, _, poll, value = dht
    poll(lambda: (21.5, 60.0))

    assert value("temperature") == 21.5
    assert value("humidity") == 60.0

    monotonic.now += ws.DHT_STALE_AFTER_SECONDS
    assert value("temperature") == 21.5  # exactly at the window: still fresh

    monotonic.now += 0.001
    assert math.isnan(value("temperature"))
    assert math.isnan(value("humidity"))


def test_stuck_reader_goes_stale_without_any_poll_returning(dht):
    # A hung read never returns, so staleness must be judged when the gauge is collected.
    _, _, monotonic, _, poll, value = dht
    poll(lambda: (18.0, 50.0))
    monotonic.now += ws.DHT_STALE_AFTER_SECONDS + 1

    assert math.isnan(value("temperature"))


def test_read_returning_none_does_not_refresh_last_success(dht):
    _, _, monotonic, wall, poll, value = dht
    poll(lambda: (21.5, 60.0))
    first_success = wall.now

    monotonic.now += ws.DHT_STALE_AFTER_SECONDS
    wall.now += ws.DHT_STALE_AFTER_SECONDS
    poll(lambda: (None, 61.0))
    poll(lambda: (22.0, None))
    monotonic.now += 1

    assert math.isnan(value("temperature"))
    assert value(TIMESTAMP, sensor="dht22") == first_success


def test_timestamp_is_wall_clock_of_last_success(dht):
    _, _, monotonic, wall, poll, value = dht
    poll(lambda: (21.5, 60.0))
    assert value(TIMESTAMP, sensor="dht22") == 1_700_000_000.0

    monotonic.now += 30
    wall.now += 30
    poll(lambda: (21.7, 59.0))
    assert value(TIMESTAMP, sensor="dht22") == 1_700_000_030.0
    with pytest.raises(OSError):
        poll(failing_read)
    assert value(TIMESTAMP, sensor="dht22") == 1_700_000_030.0


def test_dht_gauges_keep_names_and_help():
    registry = CollectorRegistry()
    ws.register_dht_metrics(ws.DhtState(), registry=registry)
    docs = {metric.name: metric.documentation for metric in registry.collect()}

    assert docs["temperature"] == "temperature in Centigrade"
    assert docs["humidity"] == "humidity percentage"
    assert TIMESTAMP in docs


def test_stale_window_is_relative_to_poll_interval():
    assert ws.DHT_STALE_AFTER_SECONDS == 5 * ws.DHT_POLL_INTERVAL_SECONDS
