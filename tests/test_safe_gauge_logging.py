import logging
import math
import sys
import threading

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


def test_concurrent_scrapes_of_a_failing_sensor_warn_once(caplog):
    # Every /metrics request runs on its own thread. Release a burst of scrapes into the
    # failing read at the same moment, with a tiny GIL switch interval so the threads
    # interleave around the failing-flag update; the outage must still warn exactly once.
    threads_per_round = 16
    rounds = 30
    old_interval = sys.getswitchinterval()
    sys.setswitchinterval(1e-6)
    try:
        for _ in range(rounds):
            caplog.clear()
            barrier = threading.Barrier(threads_per_round)

            def read(barrier=barrier):
                barrier.wait()
                raise hardware_stubs.SensorReadError("sensor gone")

            registry = CollectorRegistry()
            ws.safe_gauge("racy_metric", "fails for everyone at once", read, registry=registry)
            with caplog.at_level(logging.DEBUG, logger="weather_station"):
                workers = [
                    threading.Thread(target=lambda r=registry: list(r.collect()))
                    for _ in range(threads_per_round)
                ]
                for worker in workers:
                    worker.start()
                for worker in workers:
                    worker.join()
            warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
            assert len(warnings) == 1
    finally:
        sys.setswitchinterval(old_interval)
