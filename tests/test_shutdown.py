"""SIGINT and SIGTERM stop the exporter promptly, with exit code 0 and GPIO cleaned up."""

import signal
import subprocess
import sys
import threading

import pytest
from conftest import REPO_ROOT

RUNNER = REPO_ROOT / "tests" / "shutdown_runner.py"
STARTUP_TIMEOUT = 10
STOP_TIMEOUT = 3


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM], ids=["SIGINT", "SIGTERM"])
def test_signal_stops_exporter_and_cleans_up_gpio(signum):
    process = subprocess.Popen(
        [sys.executable, str(RUNNER)],
        cwd=REPO_ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    lines = []
    ready = threading.Event()

    def read_stdout():
        for line in process.stdout:
            lines.append(line.strip())
            if line.strip() == "ready":
                ready.set()

    reader = threading.Thread(target=read_stdout, daemon=True)
    reader.start()
    try:
        assert ready.wait(STARTUP_TIMEOUT), "exporter never became ready"
        signalled_at = len(lines)
        process.send_signal(signum)
        try:
            returncode = process.wait(STOP_TIMEOUT)
        except subprocess.TimeoutExpired:
            pytest.fail(f"exporter still running {STOP_TIMEOUT}s after {signum.name}")
        reader.join(1)
        stderr = process.stderr.read()
        assert returncode == 0, stderr
        assert "cleanup" in lines[signalled_at:], (lines, stderr)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
