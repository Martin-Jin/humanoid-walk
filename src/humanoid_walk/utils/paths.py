"""Repository folder locations and run-directory naming."""

from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
CONFIGS_DIR = REPO_ROOT / "configs"
ASSETS_DIR = REPO_ROOT / "assets"
DATA_DIR = REPO_ROOT / "data"
CHECKPOINTS_DIR = REPO_ROOT / "checkpoints"
RUNS_DIR = REPO_ROOT / "runs"


def new_run_name(prefix: str = "run") -> str:
    """Return a sortable name such as ``run-20260101-120000``."""
    return f"{prefix}-{datetime.now():%Y%m%d-%H%M%S}"


def new_run_dir(root: Path = CHECKPOINTS_DIR, prefix: str = "run") -> Path:
    """Create and return a fresh timestamped folder under ``root``."""
    path = root / new_run_name(prefix)
    path.mkdir(parents=True, exist_ok=False)
    return path
