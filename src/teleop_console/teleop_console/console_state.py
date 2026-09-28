"""Keyboard console state machine (no ROS): key -> action, one-line status."""

from __future__ import annotations

from dataclasses import dataclass

ESC = "\x1b"


@dataclass
class ConsoleState:
    seeds: list[int]
    seed_index: int = -1
    estop: bool = False
    engaged: bool = False
    message: str = "ready"

    @property
    def seed(self) -> int | None:
        return self.seeds[self.seed_index] if self.seed_index >= 0 else None

    def handle(self, key: str) -> tuple[str, dict] | None:
        """Map a key to an action (name, args) for the node to execute; None if ignored."""
        if key == ESC:
            return ("estop", {})
        if key == "c":
            return ("clear", {})
        if key == "r":
            self.seed_index = (self.seed_index + 1) % len(self.seeds)
            return ("reset", {"seed": self.seed})
        return None

    def status_line(self) -> str:
        flags = [
            "E-STOP" if self.estop else "run",
            f"scene {self.seed}" if self.seed is not None else "scene -",
            "engaged" if self.engaged else "clutch off",
        ]
        return f"{' | '.join(flags)} | {self.message}"
