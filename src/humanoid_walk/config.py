"""Load the project's configuration from the YAML files in ``configs/``."""

from pathlib import Path
from typing import Any

import yaml

from humanoid_walk.utils.paths import CONFIGS_DIR


def load_yaml(name: str, directory: Path = CONFIGS_DIR) -> dict[str, Any]:
    """Read ``directory/name`` and return its top-level mapping."""
    with open(directory / name, encoding="utf-8") as file:
        data = yaml.safe_load(file)
    if not isinstance(data, dict):
        raise ValueError(f"{name} must contain a YAML mapping at the top level.")
    return data


# TODO: frozen dataclasses for each config section, built from ``load_yaml``.
