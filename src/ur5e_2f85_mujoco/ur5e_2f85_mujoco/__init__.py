"""UR5e + Robotiq 2F-85 MuJoCo scene, kinematics oracle and shared task logic (no ROS)."""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import yaml

_PKG_DIR = Path(__file__).resolve().parent


def _data_dir() -> Path:
    """Directory holding scene.xml + model_names.yaml with ``../assets`` next to it.

    In the source tree (or a symlink install) that is this package directory. In a regular
    install the files live in ``<prefix>/share/ur5e_2f85_mujoco/ur5e_2f85_mujoco``.
    """
    if (_PKG_DIR / "scene.xml").exists() and (_PKG_DIR.parent / "assets").exists():
        return _PKG_DIR
    for parent in _PKG_DIR.parents:
        cand = parent / "share" / "ur5e_2f85_mujoco" / "ur5e_2f85_mujoco"
        if (cand / "scene.xml").exists():
            return cand
    raise FileNotFoundError(f"scene.xml not found near {_PKG_DIR} (sys.prefix={sys.prefix})")


def scene_path() -> Path:
    return _data_dir() / "scene.xml"


def model_names_path() -> Path:
    return _data_dir() / "model_names.yaml"


def load_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(scene_path()))


def load_model_names() -> dict:
    with open(model_names_path()) as f:
        return yaml.safe_load(f)
