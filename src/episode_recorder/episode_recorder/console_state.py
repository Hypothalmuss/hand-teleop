"""Keyboard console state machine (no ROS): key -> action, one-line status."""

from __future__ import annotations

from dataclasses import dataclass, field

ESC = "\x1b"


@dataclass
class ConsoleState:
    seeds: list[int]
    seed_index: int = -1
    estop: bool = False
    recording: bool = False
    success_marked: bool = False
    engaged: bool = False
    auto_success: bool = False
    message: str = "ready"
    history: list[str] = field(default_factory=list)

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
            if self.recording:
                self.message = "stop the episode before resetting"
                return None
            self.seed_index = (self.seed_index + 1) % len(self.seeds)
            self.success_marked = False
            return ("reset", {"seed": self.seed})
        if key == " ":
            if self.recording:
                return ("stop", {"success": self.success_marked, "discard": False})
            return ("start", {})
        if key == "d":
            if not self.recording:
                self.message = "no episode to discard"
                return None
            return ("stop", {"success": False, "discard": True})
        if key == "s":
            self.success_marked = not self.success_marked
            self.message = "success marked" if self.success_marked else "success unmarked"
            return None
        return None

    def status_line(self) -> str:
        flags = [
            "E-STOP" if self.estop else "run",
            "REC" if self.recording else "idle",
            f"seed {self.seed}" if self.seed is not None else "seed -",
            "engaged" if self.engaged else "clutch off",
            "SUCCESS" if self.auto_success else "",
            "[s]" if self.success_marked else "",
        ]
        text = " | ".join(f for f in flags if f)
        return f"{text} | {self.message}"
