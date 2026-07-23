from __future__ import annotations

import shutil
import uuid
from pathlib import Path

import pytest

from app.config import PROJECT_ROOT

SCRATCH = PROJECT_ROOT / ".tmp_pytest"


@pytest.fixture()
def tmp_path():
    path = SCRATCH / uuid.uuid4().hex
    path.mkdir(parents=True, exist_ok=True)
    yield path
    shutil.rmtree(path, ignore_errors=True)
