"""Tests run against the CAD-derived config only, never a local config/calibration.yaml."""
from pathlib import Path

import pytest

import umc_twin.config as config


@pytest.fixture(autouse=True)
def _no_local_calibration(monkeypatch):
    monkeypatch.setattr(config, "DEFAULT_CALIBRATION", Path("/nonexistent/calibration.yaml"))
