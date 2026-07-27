# state_machine.py — Sputter process state machine

import threading
from config import (
    IDLE_PRESSURE_MAX_VOLTAGE,
    PUMP_DOWN_COMPLETE_VOLTAGE,
)

# ── Valid manual transitions ──────────────────────────
VALID_TRANSITIONS = {
    "IDLE":            ["PUMP_DOWN"],
    "PUMP_DOWN":       ["IDLE", "VENTING"],
    "READY":           ["PUMP_DOWN", "ARGON_FLUSH", "VENTING", "IDLE"],
    "ARGON_FLUSH":     ["PLASMA_IGNITING", "READY", "VENTING", "IDLE"],
    "PLASMA_IGNITING": ["SPUTTER_READY", "VENTING", "IDLE"],
    "SPUTTER_READY":   ["SPUTTERING", "VENTING", "IDLE"],
    "SPUTTERING":      ["READY", "VENTING", "IDLE"],
    "VENTING":         ["IDLE"],
}

# ── GUI colors per state ──────────────────────────────
# Hex codes, not X11 names, for the less-universal ones (some Tk builds --
# e.g. macOS's old bundled Tk 8.5 -- don't ship the extended X11 color-name
# table; "grey"/"green" are basic/always-safe, left as names for readability).
STATE_COLORS = {
    "IDLE":            "grey",
    "PUMP_DOWN":       "#FFA500",  # orange
    "READY":           "green",
    "ARGON_FLUSH":     "#FF00FF",  # magenta
    "PLASMA_IGNITING": "#800080",  # purple
    "SPUTTER_READY":   "#40E0D0",  # turquoise
    "SPUTTERING":      "#00FFFF",  # cyan
    "VENTING":         "#FFA500",  # orange
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

        Note: ARGON_FLUSH -> PLASMA_IGNITING is owned exclusively by _poll() in
        main.py (it arms the ignition timeout and fires _ignite_plasma()),
        PLASMA_IGNITING -> SPUTTER_READY is operator-only via Confirm Plasma
        (or the operator-armed PZEM Auto-Confirm path, also in _poll()), and
        VENTING -> IDLE is owned exclusively by _poll() (gated on the operator
        confirming the primary/roughing pump is off, via vent_complete_ready()
        — reaching atmospheric pressure alone is not enough: the turbo inlet
        valve closes the instant this state is left, per turbo_valve_step(),
        and that must not happen before the operator has had a chance to
        physically switch off the primary pump) — none of these three may be
        duplicated here.
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

            # If an automatic transition occurred, broadcast it to the hardware hook
            if old_state != self._state and self.on_transition_callback:
                try:
                    self.on_transition_callback(old_state, self._state)
                except Exception as e:
                    print(f"[SM ERROR] Automatic transition callback failed: {e}")