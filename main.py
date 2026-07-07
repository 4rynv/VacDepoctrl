#!/usr/bin/env python3
"""
main.py — Sputter Vacuum Controller
State machine + live scrolling graphs for Pirani voltage and MFC flow.
"""

import math
import socket
import threading
import time
import tkinter as tk
from tkinter import messagebox

import RPi.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS
from adafruit_extended_bus import ExtendedI2C

from config import (
    ADC_GAIN,
    POLLING_INTERVAL,
    GUI_REFRESH_INTERVAL,
    ERROR_DISPLAY_SECONDS,
    GRAPH_MAX_SAMPLES,
    GRAPH_PIRANI_MIN,
    GRAPH_PIRANI_MAX,
    GRAPH_PIRANI_COLOR,
    GRAPH_MFC_MIN,
    GRAPH_MFC_MAX,
    GRAPH_MFC_COLOR,
    IDLE_PRESSURE_MAX_VOLTAGE,
    PUMP_DOWN_COMPLETE_VOLTAGE,
    ARGON_FLUSH_TARGET_VOLTAGE,
    ARGON_FLUSH_FLOW_SETPOINT,
    SPUTTER_READY_TARGET_VOLTAGE,
    PLASMA_IGNITION_TIMEOUT,
    PRESSURE_CONTROL_KP,
    ADS1115_I2C_ADDRESS,
    ARGON_DAC_VREF,
    GPIO_TURBO_VALVE_PIN,
    TURBO_VALVE_OPEN_MBAR,
    I2C_BUS_NUMBER,
)
# Pirani calibration table: (mbar, gauge_voltage) from datasheet
# ADC voltage = gauge_voltage * 0.33 (resistor divider)
_PIRANI_CAL = [
    (999, 10.00), (850, 9.58), (800, 9.52), (700, 9.45), (600, 9.38),
    (500, 9.28),  (400, 9.20), (300, 9.15), (200, 9.08), (100, 9.04),
    (50,  8.85),  (25,  8.70), (20,  8.60), (15,  8.40), (10,  8.20),
    (9,   8.10),  (8,   8.00), (7,   7.85), (6,   7.75), (5,   7.55),
    (4,   7.28),  (3,   6.92), (2,   6.27), (1,   5.70), (0.9, 5.60),
    (0.8, 5.51),  (0.7, 5.45), (0.6, 5.31), (0.5, 5.15), (0.4, 4.90),
    (0.3, 4.52),  (0.2, 4.28), (0.1, 3.98), (0.09, 3.90),(0.08, 3.75),
    (0.07, 3.64), (0.06, 3.55),(0.05, 3.44),(0.04, 3.23),(0.03, 2.93),
    (0.02, 2.50), (0.01, 1.55),(0.009, 1.45),(0.008, 1.30),(0.007, 1.10),
    (0.006, 0.95),(0.005, 0.80),(0.004, 0.65),(0.003, 0.50),(0.002, 0.35),
    (0.001, 0.12),(0.0, 0.01),
]
# Pre-scale by divider ratio so table is in ADC volts
_PIRANI_CAL_ADC = [(p, v * 0.33) for p, v in _PIRANI_CAL]

def mbar_to_adc_voltage(mbar):
    """Interpolate Pirani calibration table using log-linear interpolation.
    Interpolates linearly in log-pressure space to match the gauge physical response.
    """
    cal = _PIRANI_CAL_ADC
    if mbar >= cal[0][0]:  return cal[0][1]
    if mbar <= cal[-1][0]: return cal[-1][1]
    for i in range(len(cal) - 1):
        p_hi, v_hi = cal[i]
        p_lo, v_lo = cal[i+1]
        if p_lo <= mbar <= p_hi:
            if p_lo <= 0: p_lo = 1e-6
            t = (math.log(mbar) - math.log(p_lo)) / (math.log(p_hi) - math.log(p_lo))
            return v_lo + t * (v_hi - v_lo)
    return cal[-1][1]

def adc_voltage_to_mbar(voltage):
    """Inverse of mbar_to_adc_voltage: ADC voltage -> pressure in mbar.
    Interpolates linearly in voltage, logarithmically in pressure, matching
    the Pirani gauge's log response (same DHPG-015 calibration table).
    """
    cal = _PIRANI_CAL_ADC
    if voltage >= cal[0][1]:  return cal[0][0]   # >= 3.30 V: atmosphere (999)
    if voltage <= cal[-1][1]: return cal[-1][0]  # <= 0.0033 V: over-range (0)
    for i in range(len(cal) - 1):
        p_hi, v_hi = cal[i]
        p_lo, v_lo = cal[i+1]
        if v_lo <= voltage <= v_hi:
            if p_lo <= 0: p_lo = 1e-6
            t = (voltage - v_lo) / (v_hi - v_lo)
            return math.exp(math.log(p_lo) + t * (math.log(p_hi) - math.log(p_lo)))
    return cal[-1][0]

def turbo_valve_step(venting, mbar, valve_open, above_ticks):
    """Debounced turbo inlet valve latch. Pure decision logic — no I/O.

    The valve may only open during VENTING, and only after
    TURBO_VALVE_CONFIRM_SAMPLES consecutive readings above
    TURBO_VALVE_OPEN_MBAR: a single corrupted read on the software I2C
    bus (EMI bit-flips return garbage without raising) must not latch
    the valve open. A below-threshold reading resets the count. Once
    open, the valve stays latched open (no relay chatter) until VENTING
    is left; leaving VENTING also clears the count so a partial count
    can't carry into the next vent. Returns (valve_open, above_ticks).
    """
    if not venting:
        return False, 0
    if valve_open:
        return True, 0
    if mbar > TURBO_VALVE_OPEN_MBAR:
        above_ticks += 1
        return above_ticks >= TURBO_VALVE_CONFIRM_SAMPLES, above_ticks
    return False, 0

from pirani        import PiraniController
from mfc_control   import MFCController
from turbo_rpm     import TurboRPMController
from state_machine import SputterStateMachine, STATE_COLORS
from graph         import ScrollingGraph


# ════════════════════════════════════════════════════════
#  HARDWARE INIT
# ════════════════════════════════════════════════════════
GPIO.setmode(GPIO.BCM)

# Turbo inlet valve (GPIO 4 -> BC547 -> relay, valve on NC contact):
# LOW = relay released = NC shorted = valve CLOSED. Held closed from startup;
# driven HIGH (valve open) only above TURBO_VALVE_OPEN_MBAR during VENTING (see _poll).
GPIO.setup(GPIO_TURBO_VALVE_PIN, GPIO.OUT)
GPIO.output(GPIO_TURBO_VALVE_PIN, GPIO.LOW)

i2c = ExtendedI2C(I2C_BUS_NUMBER)
ads = ADS.ADS1115(i2c, address=ADS1115_I2C_ADDRESS)
ads.gain = ADC_GAIN

pirani    = PiraniController(ads)
mfc       = MFCController(ads, i2c=i2c)
turbo_rpm = TurboRPMController(ads)
sm        = SputterStateMachine()

def handle_hardware_interlocks(old_state, new_state):
    """Executes instantaneous safety overrides on the main thread when states shift."""
    if new_state in ["IDLE", "VENTING"]:
        # Safety cutoff: kill gas flow, drop DAC to 0V, turn off opto
        mfc.set_flow(0.0)
        mfc.valve_close()
        pirani.set_opto(False)
        # Reset plasma ignition tracking so the next flush cycle starts clean —
        # a stale triggered flag would block _poll from re-arming the ignition
        # timeout on the next ARGON_FLUSH cycle.
        with _lock:
            _state["plasma_ignition_start"] = None
            _state["plasma_ignition_triggered"] = False

    if new_state == "PUMP_DOWN":
        # Reset opto so it can re-trigger at 1.2 V on the next pump-down cycle
        pirani.set_opto(False)

# Register the hook into the state machine
sm.on_transition_callback = handle_hardware_interlocks

# ════════════════════════════════════════════════════════
#  IP ADDRESS
# ════════════════════════════════════════════════════════
def _get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "Unavailable"


# ════════════════════════════════════════════════════════
#  SHARED STATE
# ════════════════════════════════════════════════════════
_lock  = threading.Lock()
_state = {
    "pirani_adc":      0,
    "pirani_voltage":  0.0,
    "opto_enabled":    False,
    "mfc_adc":         0,
    "mfc_voltage":     0.0,
    "mfc_flow":        0.0,
    "mfc_valve_closed": False,

    "dac_ready":       False,
    "dac_voltage":     0.0,
    "dac_code":        0,

    "flow_target":     0.0,
    "turbo_valve_open": False,
    "turbo_rpm_adc":   0,
    "turbo_rpm_voltage": 0.0,
    "turbo_rpm":       0.0,
    "argon_pressure":  0.0,
    "sputter_target_mbar": 0.007,  # default 0.007 mbar
    "plasma_ignition_start": None,
    "plasma_ignition_triggered": False,
    "sm_state":        "IDLE",
    "error":           "",
}


def _on_start_argon_flush():
    with _lock:
        argon_pressure = _state["argon_pressure"]

    if argon_pressure < 15.0:
        messagebox.showwarning(
            "Argon pressure low",
            f"Argon pressure is {argon_pressure:.1f} psi. "
            "Increase it to at least 15 psi before starting argon flush."
        )
        with _lock:
            _state["error"] = "Argon pressure below 15 psi; cannot start argon flush."
        return

    if not sm.transition("ARGON_FLUSH"):
        with _lock:
            _state["error"] = "Cannot start argon flush in the current state."


def _on_start_sputter():
    with _lock:
        argon_pressure = _state["argon_pressure"]

    if argon_pressure < 15.0:
        messagebox.showwarning(
            "Argon pressure low",
            f"Argon pressure is {argon_pressure:.1f} psi. "
            "Increase it to at least 15 psi before starting sputtering."
        )
        with _lock:
            _state["error"] = "Argon pressure below 15 psi; cannot start sputtering."
        return

    if not sm.transition("SPUTTERING"):
        with _lock:
            _state["error"] = "Cannot start sputtering in the current state."

def _on_confirm_plasma():
    """Operator manually confirms plasma is ignited; proceed to SPUTTER_READY."""
    if sm.transition("SPUTTER_READY"):
        with _lock:
            _state["plasma_ignition_start"] = None
            _state["plasma_ignition_triggered"] = False
            _state["error"] = ""
    else:
        with _lock:
            _state["error"] = "Cannot confirm plasma in current state."

# ════════════════════════════════════════════════════════
#  POLLING THREAD
# ════════════════════════════════════════════════════════
_stop_event = threading.Event()

def _poll():
    turbo_valve_open = False  # Pin driven LOW (valve closed) during hardware init
    turbo_above_ticks = 0     # Consecutive VENTING reads above TURBO_VALVE_OPEN_MBAR
    while not _stop_event.is_set():
        start_time = time.time()  # Track start time for precise loop interval timing
        try:
            current = sm.state

            if current == "IDLE":
                p = pirani.read(auto_opto=False)
                pirani.set_opto(False)
                mfc.set_flow(0.0)  # Ensure DAC is driven to 0V when not in use
                mfc.valve_close()
                if p["voltage"] <= IDLE_PRESSURE_MAX_VOLTAGE:
                    sm.transition("PUMP_DOWN", pirani_voltage=p["voltage"])
            elif current == "PUMP_DOWN":
                p = pirani.read(auto_opto=False)
                # Transition-based opto: fire HIGH only when voltage first drops below 1.3 V
                if not p["opto_enabled"] and p["voltage"] <= 1.3:
                    pirani.set_opto(True)
                    p["opto_enabled"] = True
                mfc.set_flow(0.0)
                mfc.valve_close()
                # Auto return to IDLE if pressure rises back above threshold (pump failure/leak)
                if p["voltage"] >= IDLE_PRESSURE_MAX_VOLTAGE:
                    sm.transition("IDLE")
            elif current == "READY":
                p = pirani.read(auto_opto=True)
                mfc.set_flow(0.0)  # Ensure DAC is driven to 0V when not in use
                mfc.valve_close()
            elif current == "ARGON_FLUSH":
                p = pirani.read(auto_opto=True)
                mfc.valve_release()
                # PD-loop: raise flow until Pirani reaches target (0.09 mbar ≈ 1.20 V)
                mfc.pressure_control_step(p["voltage"], ARGON_FLUSH_TARGET_VOLTAGE)
                with _lock:
                    already_triggered = _state["plasma_ignition_triggered"]
                if p["voltage"] >= ARGON_FLUSH_TARGET_VOLTAGE and not already_triggered:
                    _ignite_plasma()
                    if sm.transition("PLASMA_IGNITING"):
                        with _lock:
                            _state["plasma_ignition_start"] = time.time()
                            _state["plasma_ignition_triggered"] = True
            elif current == "PLASMA_IGNITING":
                p = pirani.read(auto_opto=True)
                mfc.valve_release()
                # Hold pressure at 0.09 mbar while waiting for plasma to ignite
                mfc.pressure_control_step(p["voltage"], ARGON_FLUSH_TARGET_VOLTAGE)
                with _lock:
                    t_start = _state["plasma_ignition_start"]
                # Auto-abort if plasma not confirmed within timeout
                if t_start is not None and (time.time() - t_start) >= PLASMA_IGNITION_TIMEOUT:
                    mfc.set_flow(0.0)
                    mfc.valve_close()
                    sm.transition("READY")
                    with _lock:
                        _state["plasma_ignition_start"] = None
                        _state["plasma_ignition_triggered"] = False
                        _state["error"] = "Plasma ignition timed out; returned to READY."
            elif current == "SPUTTER_READY":
                p = pirani.read(auto_opto=True)
                mfc.valve_release()
                with _lock:
                    sputter_target_mbar = _state["sputter_target_mbar"]
                # Convert mbar to ADC voltage via calibration table
                sputter_target_voltage = mbar_to_adc_voltage(sputter_target_mbar)
                mfc.pressure_control_step(p["voltage"], sputter_target_voltage)
            elif current == "SPUTTERING":
                p = pirani.read(auto_opto=True)
                mfc.valve_release()
                with _lock:
                    sputter_target_mbar = _state["sputter_target_mbar"]
                sputter_target_voltage = mbar_to_adc_voltage(sputter_target_mbar)
                mfc.pressure_control_step(p["voltage"], sputter_target_voltage)
            elif current == "VENTING":
                p = pirani.read(auto_opto=False)
                mfc.set_flow(0.0)  # Ensure DAC is driven to 0V during venting operations
                mfc.valve_close()  # Keep MFC valve closed throughout venting
                pirani.set_opto(False)
            else:
                p = pirani.read(auto_opto=True)
                mfc.valve_release()

            # Turbo inlet valve: held CLOSED (LOW) at all times, except latched
            # OPEN (HIGH) during VENTING after TURBO_VALVE_CONFIRM_SAMPLES
            # consecutive reads above TURBO_VALVE_OPEN_MBAR (see turbo_valve_step).
            # Re-closed automatically on leaving VENTING.
            was_open = turbo_valve_open
            turbo_valve_open, turbo_above_ticks = turbo_valve_step(
                current == "VENTING", adc_voltage_to_mbar(p["voltage"]),
                turbo_valve_open, turbo_above_ticks)
            if turbo_valve_open != was_open:
                GPIO.output(GPIO_TURBO_VALVE_PIN,
                            GPIO.HIGH if turbo_valve_open else GPIO.LOW)

            m = mfc.read()
            r = turbo_rpm.read()  # Display only; read every tick regardless of state

            # Pass the raw ADC counts into the state machine to satisfy the atmosphere transition rule
            sm.update(p["voltage"], pirani_adc=p["adc"], opto_enabled=p["opto_enabled"])

            with _lock:
                _state.update({
                    "pirani_adc":       p["adc"],
                    "pirani_voltage":   p["voltage"],
                    "opto_enabled":     p["opto_enabled"],
                    "mfc_adc":          m["adc"],
                    "mfc_voltage":      m["voltage"],
                    "mfc_flow":         m["flow"],
                    "mfc_valve_closed": m.get("valve_closed", False),
                    "dac_ready":        m.get("dac_ready", False),
                    "dac_voltage":      m.get("dac_voltage", 0.0),
                    "dac_code":         m.get("dac_code", 0),
                    "flow_target":      m.get("flow_target", 0.0),
                    "turbo_valve_open": turbo_valve_open,
                    "turbo_rpm_adc":    r["adc"],
                    "turbo_rpm_voltage": r["voltage"],
                    "turbo_rpm":        r["rpm"],
                    "sm_state":         sm.state,
                })

            # Re-assert the safety cutoff if an E-STOP / vent transition landed
            # mid-tick: the branch above read `current` before the transition and
            # may have called valve_release()/set_flow() *after* the interlock
            # callback already cut everything off. Last write must be the cutoff.
            if sm.state in ("IDLE", "VENTING") and current not in ("IDLE", "VENTING"):
                mfc.set_flow(0.0)
                mfc.valve_close()
                pirani.set_opto(False)

        except Exception as e:
            # Prevents background thread termination if an I2C transaction glitches
            print(f"[HW ERROR] Polling loop glitch (likely EMI): {e}")
            with _lock:
                _state["error"] = str(e)

        # Enforce consistent execution timing based on configurations
        elapsed = time.time() - start_time
        sleep_time = max(0.01, POLLING_INTERVAL - elapsed)
        time.sleep(sleep_time)


_poll_thread = threading.Thread(target=_poll, daemon=True)
_poll_thread.start()


# ════════════════════════════════════════════════════════
#  TKINTER GUI
# ════════════════════════════════════════════════════════
root = tk.Tk()
root.title("Sputter Vacuum Controller")
root.resizable(False, False)
root.columnconfigure(0, weight=1)
root.columnconfigure(1, weight=1)

# ── IP bar ───────────────────────────────────────────
ip_frame = tk.Frame(root)
ip_frame.grid(row=0, column=0, columnspan=2, padx=12, pady=(10, 0), sticky="ew")
tk.Label(ip_frame, text="Pi IP  :", font=("Courier", 11, "bold")).pack(side="left")
tk.Label(ip_frame, text=_get_ip(), font=("Courier", 11), fg="blue").pack(side="left")

# ── State display ─────────────────────────────────────
sf = tk.LabelFrame(root, text=" Process State ", padx=10, pady=8)
sf.grid(row=1, column=0, padx=12, pady=(10, 6), sticky="ew")
lbl_state = tk.Label(sf, text="IDLE", font=("Courier", 18, "bold"),
                     width=16, relief="sunken", fg="grey")
lbl_state.grid(row=0, column=0, columnspan=2)

# ── Control buttons ───────────────────────────────────
bf = tk.LabelFrame(root, text=" Controls ", padx=10, pady=8)
bf.grid(row=2, column=0, padx=12, pady=(0, 6), sticky="ew")

btn_flush = tk.Button(
    bf,
    text="Start Argon Flush",
    width=16,
    font=("Courier", 10),
    command=_on_start_argon_flush
)

btn_confirm_plasma = tk.Button(
    bf,
    text="Confirm Plasma",
    width=16,
    font=("Courier", 10),
    command=_on_confirm_plasma
)

btn_sputter = tk.Button(
    bf,
    text="Start Sputter",
    width=16,
    font=("Courier", 10),
    command=_on_start_sputter
)

btn_stop = tk.Button(
    bf,
    text="Stop Sputter",
    width=16,
    font=("Courier", 10),
    command=lambda: sm.transition("VENTING")
)

btn_vent = tk.Button(
    bf,
    text="Vent",
    width=16,
    font=("Courier", 10),
    command=lambda: sm.transition("VENTING")
)

btn_estop = tk.Button(
    bf,
    text="E-STOP",
    width=16,
    font=("Courier", 10, "bold"),
    fg="white",
    bg="red",
    command=sm.emergency_stop
)

btn_flush          .grid(row=0, column=0, padx=4, pady=4)
btn_confirm_plasma .grid(row=0, column=1, padx=4, pady=4)
btn_sputter        .grid(row=1, column=0, padx=4, pady=4)
btn_stop           .grid(row=1, column=1, padx=4, pady=4)
btn_vent           .grid(row=2, column=0, padx=4, pady=4)
btn_estop          .grid(row=3, column=0, columnspan=2, padx=4, pady=(4, 0), sticky="ew")

# ── Pirani frame ──────────────────────────────────────
pf = tk.LabelFrame(root, text=" Pirani Gauge  [A0] ", padx=10, pady=6)
pf.grid(row=3, column=0, padx=12, pady=(6, 6), sticky="ew")

lbl_p_adc      = tk.Label(pf, text="ADC     : ——",      font=("Courier", 12), anchor="w", width=32)
lbl_p_volt     = tk.Label(pf, text="Voltage : ——.—— V", font=("Courier", 12), anchor="w", width=32)
lbl_p_mbar     = tk.Label(pf, text="Pressure: ——.—— mbar", font=("Courier", 12, "bold"), anchor="w", width=32)
lbl_opto       = tk.Label(pf, text="OPTO    : ——",      font=("Courier", 14, "bold"), anchor="w", width=32)
lbl_tvalve     = tk.Label(pf, text="T-VALVE : ——",      font=("Courier", 14, "bold"), anchor="w", width=32)
lbl_turbo_rpm  = tk.Label(pf, text="T-RPM   : ——.—— RPM", font=("Courier", 12), anchor="w", width=32)
lbl_p_message  = tk.Label(pf, text="",               font=("Courier", 10), anchor="w", width=80, fg="blue")
lbl_p_adc .grid(row=0, sticky="w")
lbl_p_volt.grid(row=1, sticky="w")
lbl_p_mbar.grid(row=2, sticky="w")
lbl_opto  .grid(row=3, sticky="w", pady=(6, 0))
lbl_tvalve.grid(row=4, sticky="w")
lbl_turbo_rpm.grid(row=5, sticky="w")
lbl_p_message.grid(row=6, sticky="w", pady=(6, 0))

p_canvas = tk.Canvas(pf, height=90, bg="#1a1a1a", highlightthickness=0)
p_canvas.grid(row=7, column=0, sticky="ew", pady=(6, 2))
pf.columnconfigure(0, weight=1)

p_graph = ScrollingGraph(p_canvas, maxlen=GRAPH_MAX_SAMPLES,
                         min_val=GRAPH_PIRANI_MIN, max_val=GRAPH_PIRANI_MAX,
                         line_color=GRAPH_PIRANI_COLOR, unit="V")

# ── MFC frame ─────────────────────────────────────────
mff = tk.LabelFrame(root, text=" MFC  Argon Flow  [A1] ", padx=10, pady=6)
mff.grid(row=4, column=0, padx=12, pady=(0, 6), sticky="ew")

lbl_m_adc    = tk.Label(mff, text="ADC     : ——",            font=("Courier", 12), anchor="w", width=32)
lbl_m_volt   = tk.Label(mff, text="Voltage : ——.—— V",       font=("Courier", 12), anchor="w", width=32)
lbl_m_flow   = tk.Label(mff, text="Flow    : ——.—— sccm Ar", font=("Courier", 14, "bold"), anchor="w", width=32)
lbl_m_dac    = tk.Label(mff, text="DAC     : UNKNOWN",      font=("Courier", 12), anchor="w", width=32)
lbl_m_valve  = tk.Label(mff, text="Valve   : RELEASED",      font=("Courier", 14, "bold"), anchor="w", width=32)
lbl_m_sputter_target  = tk.Label(mff, text="Sputter P (mbar):", font=("Courier", 12), anchor="w")
entry_sputter_target  = tk.Entry(mff, width=10, font=("Courier", 12))
btn_sputter_target    = tk.Button(mff, text="Set", font=("Courier", 10), command=lambda: _update_sputter_target())
lbl_sputter_set_val   = tk.Label(mff, text="Set: 0.007 mbar", font=("Courier", 10), fg="black", anchor="w")
lbl_m_argon  = tk.Label(mff, text="Argon PSI:",            font=("Courier", 12), anchor="w")
entry_argon  = tk.Entry(mff, width=10, font=("Courier", 12))
btn_argon    = tk.Button(mff, text="Update", font=("Courier", 10), command=lambda: _update_argon_pressure())
lbl_argon_set_val     = tk.Label(mff, text="Set: 0.0 psi", font=("Courier", 10), fg="black", anchor="w")

lbl_m_adc   .grid(row=0, sticky="w")
lbl_m_volt  .grid(row=1, sticky="w")
lbl_m_flow  .grid(row=2, sticky="w", pady=(6, 0))
lbl_m_dac   .grid(row=3, sticky="w", pady=(6, 0))
lbl_m_valve .grid(row=4, sticky="w", pady=(6, 0))

lbl_m_sputter_target.grid(row=5, column=0, sticky="w", pady=(6, 0))
entry_sputter_target.grid(row=5, column=1, sticky="w", pady=(6, 0))
btn_sputter_target.grid(row=5, column=2, padx=(6, 0), pady=(6, 0))
lbl_sputter_set_val.grid(row=5, column=3, sticky="w", padx=(8, 0), pady=(6, 0))
entry_sputter_target.bind("<Return>", lambda e: _update_sputter_target())
lbl_m_argon.grid(row=6, column=0, sticky="w", pady=(6, 0))
entry_argon.grid(row=6, column=1, sticky="w", pady=(6, 0))
btn_argon.grid(row=6, column=2, padx=(6, 0), pady=(6, 0))
lbl_argon_set_val.grid(row=6, column=3, sticky="w", padx=(8, 0), pady=(6, 0))
entry_argon.bind("<Return>", lambda e: _update_argon_pressure())

m_canvas = tk.Canvas(mff, height=90, bg="#1a1a1a", highlightthickness=0)
m_canvas.grid(row=9, column=0, columnspan=4, sticky="ew", pady=(6, 2))
mff.columnconfigure(0, weight=1)

# Flow (solid, cyan, left-scale sccm) overlaid with DAC output voltage
# (dashed, yellow, right-scale volts) on the same canvas.
m_graph = ScrollingGraph(m_canvas, maxlen=GRAPH_MAX_SAMPLES,
                         min_val=GRAPH_MFC_MIN, max_val=GRAPH_MFC_MAX,
                         line_color=GRAPH_MFC_COLOR, unit="sccm",
                         secondary_min=0.0, secondary_max=ARGON_DAC_VREF,
                         secondary_color="yellow", secondary_unit="V DAC")

# ── Plasma frame ──────────────────────────────────────
plf = tk.LabelFrame(root, text=" Plasma ", padx=10, pady=6)
plf.grid(row=1, column=1, rowspan=4, padx=12, pady=(10, 6), sticky="nsew")
plf.columnconfigure(0, weight=1)
plasma_placeholder = tk.Label(
    plf,
    text="Plasma controls and status\nwill appear here later.",
    font=("Courier", 12),
    fg="grey",
    justify="left",
    anchor="nw",
)
plasma_placeholder.pack(fill="both", expand=True, padx=10, pady=10)

# ── Status bar ────────────────────────────────────────
lbl_status = tk.Label(root, text="OK", font=("Courier", 10),
                      anchor="w", width=32, fg="grey")
lbl_status.grid(row=5, column=0, columnspan=2, padx=12, pady=(0, 10), sticky="w")


# ════════════════════════════════════════════════════════
#  BUTTON STATE MAP
# ════════════════════════════════════════════════════════
# Button order: flush, confirm_plasma, sputter, stop, vent, estop
BUTTON_STATES = {
    "IDLE":            ("disabled", "disabled", "disabled", "disabled", "disabled", "normal"),
    "PUMP_DOWN":       ("disabled", "disabled", "disabled", "disabled", "disabled", "normal"),
    "READY":           ("normal",   "disabled", "disabled", "disabled", "normal",   "normal"),
    "ARGON_FLUSH":     ("disabled", "disabled", "disabled", "disabled", "normal",   "normal"),
    "PLASMA_IGNITING": ("disabled", "normal",   "disabled", "disabled", "normal",   "normal"),
    "SPUTTER_READY":   ("disabled", "disabled", "normal",   "normal",   "normal",   "normal"),
    "SPUTTERING":      ("disabled", "disabled", "disabled", "normal",   "normal",   "normal"),
    "VENTING":         ("disabled", "disabled", "disabled", "disabled", "disabled", "normal"),
}


# ════════════════════════════════════════════════════════
#  GUI REFRESH
# ════════════════════════════════════════════════════════

def _update_sputter_target():
    raw = entry_sputter_target.get().strip()
    try:
        v = float(raw)
    except ValueError:
        with _lock:
            _state["error"] = "Invalid sputter target; enter pressure in mbar."
        return
    if v <= 0.0:
        with _lock:
            _state["error"] = "Sputter target must be > 0 mbar."
        return
    with _lock:
        _state["sputter_target_mbar"] = v
        _state["error"] = ""
    lbl_sputter_set_val.config(text=f"Set: {v:.4f} mbar")

def _update_argon_pressure():
    raw_value = entry_argon.get().strip()
    try:
        value = float(raw_value)
    except ValueError:
        with _lock:
            _state["error"] = "Invalid argon pressure; enter a number in psi."
        return

    with _lock:
        _state["argon_pressure"] = value
        _state["error"] = ""
    lbl_argon_set_val.config(text=f"Set: {value:.1f} psi")


def _ignite_plasma():
    """Trigger plasma ignition via control signal.

    Integrates with RF power supply, pressure feedback, and safety interlocks.
    Placeholder for hardware control implementation.
    """
    # TODO: implement RF trigger/control logic when hardware is defined
    pass


# Tracks the currently displayed error and when it first appeared (GUI thread only)
_error_display = {"text": "", "since": 0.0}


def _refresh():
    with _lock:
        s = dict(_state)

    # — State —
    st = s["sm_state"]
    lbl_state.config(text=st, fg=STATE_COLORS.get(st, "grey"))

    # — Buttons —
    bs = BUTTON_STATES.get(st, BUTTON_STATES["IDLE"])
    btn_flush          .config(state=bs[0])
    btn_confirm_plasma .config(state=bs[1])
    btn_sputter        .config(state=bs[2])
    btn_stop           .config(state=bs[3])
    btn_vent           .config(state=bs[4])
    btn_estop          .config(state=bs[5])

    # — Pirani —
    lbl_p_adc .config(text=f"ADC     : {s['pirani_adc']}")
    lbl_p_volt.config(text=f"Voltage : {s['pirani_voltage']:.6f} V")
    mbar = adc_voltage_to_mbar(s["pirani_voltage"])
    if mbar >= 999:
        lbl_p_mbar.config(text="Pressure: ATM (>999 mbar)")
    else:
        lbl_p_mbar.config(text=f"Pressure: {mbar:.3g} mbar")
    lbl_opto  .config(
        text = "OPTO    : ON " if s["opto_enabled"] else "OPTO    : OFF",
        fg   = "green"         if s["opto_enabled"] else "red",
    )
    lbl_tvalve.config(
        text = "T-VALVE : OPEN (venting)" if s["turbo_valve_open"] else "T-VALVE : CLOSED",
        fg   = "orange"                   if s["turbo_valve_open"] else "green",
    )
    lbl_turbo_rpm.config(text=f"T-RPM   : {s['turbo_rpm']:.0f} RPM")

    if st == "IDLE":
        p_message = "IDLE: MFC valve closed, turbo opto off."
    elif st == "PUMP_DOWN":
        if s["opto_enabled"]:
            p_message = "Turbo opto ON at {:.4f} V; pump down continues.".format(s["pirani_voltage"])
        else:
            p_message = "Pump down in progress; turbo opto will enable when pressure drops sufficiently."
    elif st == "READY":
        p_message = "READY: Start Argon Flush once inlet pressure is ≥ 15 psi."
    elif st == "ARGON_FLUSH":
        if s["pirani_voltage"] >= ARGON_FLUSH_TARGET_VOLTAGE:
            p_message = "Target 0.09 mbar reached; igniting plasma automatically."
        else:
            p_message = "Argon flush: controlling flow to reach 0.09 mbar (now {:.3f} V).".format(s["pirani_voltage"])
    elif st == "PLASMA_IGNITING":
        elapsed = time.time() - s["plasma_ignition_start"] if s["plasma_ignition_start"] else 0
        remaining = max(0, int(PLASMA_IGNITION_TIMEOUT - elapsed))
        p_message = f"Waiting for plasma ignition ({remaining}s remaining). Confirm or wait for auto-abort."
    elif st == "SPUTTER_READY":
        p_message = "SPUTTER READY: Plasma stable at target pressure. Press Start Sputter."
    else:
        p_message = ""

    lbl_p_message.config(text=p_message)
    p_graph.push(s["pirani_voltage"])
    p_graph.draw()

    # — MFC —
    lbl_m_adc .config(text=f"ADC     : {s['mfc_adc']}")
    lbl_m_volt.config(text=f"Voltage : {s['mfc_voltage']:.6f} V")
    lbl_m_flow.config(text=f"Flow    : {s['mfc_flow']:.2f} sccm Ar")
    lbl_m_dac.config(
        text=f"DAC OUT : {s['dac_voltage']:.3f} V ({s['dac_code']})",
        fg   = "green" if s.get('dac_ready') else "red",
    )
    lbl_m_valve.config(
        text = "Valve   : CLOSED" if s['mfc_valve_closed'] else "Valve   : RELEASED",
        fg   = "red" if s['mfc_valve_closed'] else "green",
    )
    m_graph.push(
        s['mfc_flow'],
        s['dac_voltage']
    )
    m_graph.draw()

    # — Status —
    # Errors are no longer wiped by the polling thread; expire them here after
    # ERROR_DISPLAY_SECONDS so the operator has time to read them but a single
    # transient glitch doesn't leave the status bar red forever.
    if s["error"]:
        now = time.time()
        if s["error"] != _error_display["text"]:
            _error_display["text"]  = s["error"]
            _error_display["since"] = now
        elif now - _error_display["since"] >= ERROR_DISPLAY_SECONDS:
            with _lock:
                if _state["error"] == s["error"]:
                    _state["error"] = ""
            s["error"] = ""
            _error_display["text"] = ""
    else:
        _error_display["text"] = ""

    lbl_status.config(
        text = f"ERR: {s['error']}" if s["error"] else "OK",
        fg   = "red"                if s["error"] else "grey",
    )

    root.after(GUI_REFRESH_INTERVAL, _refresh)


# ════════════════════════════════════════════════════════
#  CLEAN SHUTDOWN
# ════════════════════════════════════════════════════════
def _on_close():
    _stop_event.set()
    _poll_thread.join(timeout=2.0)  # Wait for poll thread to exit cleanly
    mfc.set_flow(0.0)
    mfc.valve_close()
    pirani.shutdown()
    GPIO.cleanup()
    root.destroy()


root.protocol("WM_DELETE_WINDOW", _on_close)

_refresh()
root.mainloop()
