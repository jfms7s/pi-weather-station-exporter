import math
import threading
import time

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


def test_scrape_returns_nan_while_the_poll_thread_is_stuck_in_a_read(dht):
    _, _, monotonic, _, poll, value = dht
    poll(lambda: (18.0, 50.0))

    release = threading.Event()
    entered = threading.Event()

    def stuck_read():
        entered.set()
        release.wait(5)
        return 19.0, 51.0

    poller = threading.Thread(target=poll, args=(stuck_read,), daemon=True)
    poller.start()
    assert entered.wait(1)
    monotonic.now += ws.DHT_STALE_AFTER_SECONDS + 1

    scraped = {}
    scraper = threading.Thread(
        target=lambda: scraped.update(temperature=value("temperature")), daemon=True
    )
    scraper.start()
    scraper.join(1)
    try:
        assert not scraper.is_alive(), "scrape blocked behind the stuck read"
        assert math.isnan(scraped["temperature"])
    finally:
        release.set()
        poller.join(1)


def test_start_dht_poller_wires_the_loop_to_the_exported_state():
    registry = CollectorRegistry()
    polled = threading.Event()

    def read():
        polled.set()
        return 21.5, 60.0

    def submit(fn, *args):
        threading.Thread(target=fn, args=args, daemon=True).start()

    ws.start_dht_poller(submit, registry=registry, read_fn=read)

    assert polled.wait(1)
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline and math.isnan(registry.get_sample_value("temperature")):
        time.sleep(0.01)
    assert registry.get_sample_value("temperature") == 21.5
    assert registry.get_sample_value("humidity") == 60.0
