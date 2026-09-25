"""Locate and load the repo-level ``config/*.yaml`` files (the single source of truth).

Search order: ``$HAND_TELEOP_CONFIG``, a ``config/`` directory above this file (source tree or
symlink install), then ``<prefix>/share/hand_teleop_bringup/config`` for every prefix in
``$AMENT_PREFIX_PATH``.
"""

from __future__ import annotations

import os
from pathlib import Path

import yaml


def find_config_dir(explicit: str | os.PathLike | None = None) -> Path:
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    if os.environ.get("HAND_TELEOP_CONFIG"):
        candidates.append(Path(os.environ["HAND_TELEOP_CONFIG"]))
    candidates += [p / "config" for p in Path(__file__).resolve().parents]
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / "share" / "hand_teleop_bringup" / "config")
    for cand in candidates:
        if (cand / "rates.yaml").exists():
            return cand
    raise FileNotFoundError("config/rates.yaml not found; set HAND_TELEOP_CONFIG")


def load_config(name: str, config_dir: str | os.PathLike | None = None) -> dict:
    with open(find_config_dir(config_dir) / name) as f:
        return yaml.safe_load(f)
