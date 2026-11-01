import logging
import math

import hardware_stubs
from prometheus_client import CollectorRegistry, generate_latest

import Weather_Station as ws


def scrape_values(registry, name, times):
    """Collect the registry `times` times (one read per scrape) and return `name`'s values."""
    return [
        sample.value
        for _ in range(times)
        for metric in registry.collect()
        for sample in metric.samples
        if sample.name == name
    ]


def test_failing_sensor_warns_on_first_failure_and_logs_recovery(caplog):
    # 5 failing scrapes, 1 good one, then a failing one again.
    outcomes = iter([False] * 5 + [True] + [False])

    def read():
        if next(outcomes):
            return 7.0
        raise hardware_stubs.SensorReadError("sensor gone")

    registry = CollectorRegistry()
    ws.safe_gauge("flaky_metric", "fails, recovers, fails", read, registry=registry)

    with caplog.at_level(logging.DEBUG, logger="weather_station"):
        values = scrape_values(registry, "flaky_metric", 7)

    assert [math.isnan(v) for v in values] == [True] * 5 + [False] + [True]
    assert values[5] == 7.0

    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 2
    assert all(r.exc_info for r in warnings)
    recoveries = [
        r for r in caplog.records if r.levelno == logging.INFO and "recovered" in r.getMessage()
    ]
    assert len(recoveries) == 1
    assert "flaky_metric" in recoveries[0].getMessage()


def test_failing_gauge_is_nan_in_exposition_on_every_scrape():
    def broken():
        raise hardware_stubs.SensorReadError("sensor gone")

    registry = CollectorRegistry()
    ws.safe_gauge("broken_metric", "always fails", broken, registry=registry)

    for _ in range(3):
        assert b"broken_metric NaN" in generate_latest(registry)
