# state_machine.py — Sputter process state machine

import threading
from config import (
    VACUUM_THRESHOLD,
    ATMOSPHERE_THRESHOLD,
    IDLE_PRESSURE_MAX_VOLTAGE,
    PUMP_DOWN_COMPLETE_VOLTAGE,
    ARGON_FLUSH_TARGET_VOLTAGE,
    SPUTTER_READY_TARGET_VOLTAGE,
)

# ── Valid manual transitions ──────────────────────────
VALID_TRANSITIONS = {
    "IDLE":         ["PUMP_DOWN"],
    "PUMP_DOWN":    ["IDLE"],
    "READY":          ["ARGON_FLUSH", "VENTING", "IDLE"],
    "ARGON_FLUSH":     ["PLASMA_IGNITING", "READY", "IDLE"],
    "PLASMA_IGNITING": ["SPUTTER_READY", "VENTING", "IDLE"],
    "SPUTTER_READY":   ["SPUTTERING", "VENTING", "IDLE"],
    "SPUTTERING":      ["READY", "VENTING", "IDLE"],
    "VENTING":         ["IDLE"],
}

# ── GUI colors per state ──────────────────────────────
STATE_COLORS = {
    "IDLE":         "grey",
    "PUMP_DOWN":    "orange",
    "READY":        "green",
    "ARGON_FLUSH":     "magenta",
    "PLASMA_IGNITING": "purple",
    "SPUTTER_READY":   "turquoise",
    "SPUTTERING":      "cyan",
    "VENTING":         "orange",
}


class SputterStateMachine:

    def __init__(self):
        self._state = "IDLE"
        self._lock  = threading.Lock()

    @property
    def state(self):
        with self._lock:
            return self._state

    def transition(self, new_state, pirani_voltage=None):
        """Operator-triggered transition. Returns True if allowed."""
        with self._lock:
            if new_state == "PUMP_DOWN" and self._state == "IDLE":
                if pirani_voltage is None or pirani_voltage > IDLE_PRESSURE_MAX_VOLTAGE:
                    return False

            if new_state in VALID_TRANSITIONS.get(self._state, []):
                self._state = new_state
                return True
            return False

    def emergency_stop(self):
        """Force IDLE from any state."""
        with self._lock:
            self._state = "IDLE"

    def update(self, pirani_voltage, opto_enabled=False):
        """Automatic sensor-driven transitions. Called from polling thread."""
        with self._lock:
            if self._state == "PUMP_DOWN":
                if pirani_voltage <= PUMP_DOWN_COMPLETE_VOLTAGE and opto_enabled:
                    self._state = "READY"
            elif self._state == "ARGON_FLUSH":
                if pirani_voltage >= ARGON_FLUSH_TARGET_VOLTAGE:
                    self._state = "PLASMA_IGNITING"
            elif self._state == "PLASMA_IGNITING":
                if pirani_voltage <= SPUTTER_READY_TARGET_VOLTAGE:
                    self._state = "SPUTTER_READY"
            elif self._state == "VENTING" and pirani_voltage >= ATMOSPHERE_THRESHOLD:
                self._state = "IDLE"
