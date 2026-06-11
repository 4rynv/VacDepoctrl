# state_machine.py — Sputter process state machine

import threading

# ── Automatic transition thresholds ──────────────────
VACUUM_THRESHOLD     = 10500   # ADC ≤ this → vacuum achieved  (PUMP_DOWN → READY)
ATMOSPHERE_THRESHOLD = 30000   # ADC ≥ this → back to atmo     (VENTING   → IDLE)

# ── Valid manual transitions ──────────────────────────
VALID_TRANSITIONS = {
    "IDLE":       ["PUMP_DOWN"],
    "PUMP_DOWN":  ["IDLE"],
    "READY":      ["SPUTTERING", "VENTING", "IDLE"],
    "SPUTTERING": ["READY", "IDLE"],
    "VENTING":    ["IDLE"],
}

# ── GUI colors per state ──────────────────────────────
STATE_COLORS = {
    "IDLE":       "grey",
    "PUMP_DOWN":  "orange",
    "READY":      "green",
    "SPUTTERING": "cyan",
    "VENTING":    "orange",
}


class SputterStateMachine:

    def __init__(self):
        self._state = "IDLE"
        self._lock  = threading.Lock()

    @property
    def state(self):
        with self._lock:
            return self._state

    def transition(self, new_state):
        """Operator-triggered transition. Returns True if allowed."""
        with self._lock:
            if new_state in VALID_TRANSITIONS.get(self._state, []):
                self._state = new_state
                return True
            return False

    def emergency_stop(self):
        """Force IDLE from any state."""
        with self._lock:
            self._state = "IDLE"

    def update(self, pirani_adc):
        """Automatic sensor-driven transitions. Called from polling thread."""
        with self._lock:
            if self._state == "PUMP_DOWN" and pirani_adc <= VACUUM_THRESHOLD:
                self._state = "READY"
            elif self._state == "VENTING" and pirani_adc >= ATMOSPHERE_THRESHOLD:
                self._state = "IDLE"
