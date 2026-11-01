VENV ?= .venv
PYTHON ?= python3

.PHONY: install lock

# Create the project virtualenv and install the hash-locked dependency set into it.
# Run as the unprivileged user that runs the exporter; never with sudo.
install:
	$(PYTHON) -m venv $(VENV)
	$(VENV)/bin/pip install --require-hashes -r requirements.txt

# Re-resolve requirements.in into the hashed requirements.txt (needs uv).
lock:
	uv pip compile --universal --generate-hashes --python-version 3.11 requirements.in -o requirements.txt
