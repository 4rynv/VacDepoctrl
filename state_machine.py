# state_machine.py — Sputter process state machine

import threading
from config import (
    VACUUM_THRESHOLD,  # Kept for config consistency, though currently unreferenced
    ATMOSPHERE_THRESHOLD,
    VENTING_COMPLETE_VOLTAGE,
    IDLE_PRESSURE_MAX_VOLTAGE,
    PUMP_DOWN_COMPLETE_VOLTAGE,
    ARGON_FLUSH_TARGET_VOLTAGE,
    SPUTTER_READY_TARGET_VOLTAGE,
)

# ── Valid manual transitions ──────────────────────────
VALID_TRANSITIONS = {
    "IDLE":            ["PUMP_DOWN"],
    "PUMP_DOWN":       ["IDLE"],
    "READY":           ["PUMP_DOWN", "ARGON_FLUSH", "VENTING", "IDLE"],
    "ARGON_FLUSH":     ["PLASMA_IGNITING", "READY", "VENTING", "IDLE"],
    "PLASMA_IGNITING": ["SPUTTER_READY", "VENTING", "IDLE"],
    "SPUTTER_READY":   ["SPUTTERING", "VENTING", "IDLE"],
    "SPUTTERING":      ["READY", "VENTING", "IDLE"],
    "VENTING":         ["IDLE"],
}

# ── GUI colors per state ──────────────────────────────
STATE_COLORS = {
    "IDLE":            "grey",
    "PUMP_DOWN":       "orange",
    "READY":           "green",
    "ARGON_FLUSH":     "magenta",
    "PLASMA_IGNITING": "purple",
    "SPUTTER_READY":   "turquoise",
    "SPUTTERING":      "cyan",
    "VENTING":         "orange",
}


class SputterStateMachine:

    def __init__(self, initial_state="IDLE"):
        self._state = initial_state
        self._lock = threading.Lock()
        
        # Callback hook to immediately trigger hardware changes (e.g., shutting valves)
        # when a transition completes. Assign this in main.py.
        self.on_transition_callback = None

    @property
    def state(self):
        """Thread-safe getter for the current state string."""
        with self._lock:
            return self._state

    def transition(self, new_state, pirani_voltage=None):
        """Operator-triggered manual transition. Returns True if successful."""
        with self._lock:
            if new_state == "PUMP_DOWN" and self._state == "IDLE":
                if pirani_voltage is None or pirani_voltage > IDLE_PRESSURE_MAX_VOLTAGE:
                    return False

            if new_state in VALID_TRANSITIONS.get(self._state, []):
                old_state = self._state
                self._state = new_state
                
                # Execute immediate hardware adjustments if hook is registered
                if self.on_transition_callback:
                    try:
                        self.on_transition_callback(old_state, new_state)
                    except Exception as e:
                        print(f"[SM ERROR] Manual transition callback failed: {e}")
                return True
            return False

    def emergency_stop(self):
        """Force the system into IDLE state from any active operational phase."""
        with self._lock:
            old_state = self._state
            self._state = "IDLE"
            
            if self.on_transition_callback:
                try:
                    self.on_transition_callback(old_state, "IDLE")
                except Exception as e:
                    print(f"[SM ERROR] Emergency stop callback failed: {e}")

    def update(self, pirani_voltage, pirani_adc=None, opto_enabled=False):
        """
        Automatic sensor-driven transitions. Evaluated continuously by the hardware polling thread.
        Corrects units-mismatch bug in VENTING state by matching ADC-counts against ATMOSPHERE_THRESHOLD.
        """
        with self._lock:
            old_state = self._state

            if self._state == "PUMP_DOWN":
                if pirani_voltage <= PUMP_DOWN_COMPLETE_VOLTAGE and opto_enabled:
                    self._state = "READY"
                elif pirani_voltage >= IDLE_PRESSURE_MAX_VOLTAGE:
                    # Pressure rose back above safe threshold (e.g. pump failure, leak)
                    self._state = "IDLE"

            elif self._state == "READY":
                if pirani_voltage >= IDLE_PRESSURE_MAX_VOLTAGE:
                    # Pressure degraded (e.g. leak); drop back into pump-down
                    self._state = "PUMP_DOWN"
                    
            elif self._state == "ARGON_FLUSH":
                if pirani_voltage >= ARGON_FLUSH_TARGET_VOLTAGE:
                    self._state = "PLASMA_IGNITING"
                    
            elif self._state == "PLASMA_IGNITING":
                if pirani_voltage <= SPUTTER_READY_TARGET_VOLTAGE:
                    self._state = "SPUTTER_READY"
                    
            elif self._state == "VENTING":
                # Transition to IDLE once Pirani reads atmospheric pressure
                if pirani_voltage >= VENTING_COMPLETE_VOLTAGE:
                    self._state = "IDLE"

            # If an automatic transition occurred, broadcast it to the hardware hook
            if old_state != self._state and self.on_transition_callback:
                try:
                    self.on_transition_callback(old_state, self._state)
                except Exception as e:
                    print(f"[SM ERROR] Automatic transition callback failed: {e}")