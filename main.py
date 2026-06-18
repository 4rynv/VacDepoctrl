#!/usr/bin/env python3
"""
main.py — Sputter Vacuum Controller
State machine + live scrolling graphs for Pirani voltage and MFC flow.
"""

import socket
import threading
import time
import tkinter as tk
from tkinter import messagebox

import board
import busio
import RPi.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS

from config import (
    ADC_GAIN,
    POLLING_INTERVAL,
    GUI_REFRESH_INTERVAL,
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
    ADS1115_I2C_ADDRESS,
    ARGON_DAC_VREF,
)
from pirani        import PiraniController
from mfc_control   import MFCController
from state_machine import SputterStateMachine, STATE_COLORS
from graph         import ScrollingGraph


# ════════════════════════════════════════════════════════
#  HARDWARE INIT
# ════════════════════════════════════════════════════════
GPIO.setmode(GPIO.BCM)

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c, address=ADS1115_I2C_ADDRESS)
ads.gain = ADC_GAIN

pirani = PiraniController(ads)
mfc    = MFCController(ads, i2c=i2c)
sm     = SputterStateMachine()


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
    "argon_pressure":  0.0,
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

# ════════════════════════════════════════════════════════
#  POLLING THREAD
# ════════════════════════════════════════════════════════
def _poll():
    while True:
        try:
            current = sm.state

            if current == "IDLE":
                p = pirani.read(auto_opto=False)
                pirani.set_opto(False)
                mfc.valve_close()
                if p["voltage"] <= IDLE_PRESSURE_MAX_VOLTAGE:
                    sm.transition("PUMP_DOWN", pirani_voltage=p["voltage"])
            elif current == "PUMP_DOWN":
                p = pirani.read(auto_opto=False)
                if p["voltage"] <= IDLE_PRESSURE_MAX_VOLTAGE and not p["opto_enabled"]:
                    pirani.set_opto(True)
                    p["opto_enabled"] = True
                mfc.valve_close()
            elif current == "READY":
                p = pirani.read(auto_opto=True)
                mfc.valve_close()
            elif current == "ARGON_FLUSH":
                p = pirani.read(auto_opto=True)
                mfc.set_flow(ARGON_FLUSH_FLOW_SETPOINT)
                mfc.valve_release()
                if p["voltage"] >= ARGON_FLUSH_TARGET_VOLTAGE:
                    _ignite_plasma()
                    sm.transition("PLASMA_IGNITING")
            elif current == "PLASMA_IGNITING":
                p = pirani.read(auto_opto=True)
                mfc.valve_release()
                if p["voltage"] <= SPUTTER_READY_TARGET_VOLTAGE:
                    sm.transition("SPUTTER_READY")
            elif current == "SPUTTER_READY":
                p = pirani.read(auto_opto=True)
                mfc.valve_release()
            elif current == "SPUTTERING":
                p = pirani.read(auto_opto=True)
                mfc.valve_release()
            elif current == "VENTING":
                p = pirani.read(auto_opto=False)
                mfc.valve_release()
                pirani.set_opto(False)
            else:
                p = pirani.read(auto_opto=True)
                mfc.valve_release()

            m = mfc.read()
            sm.update(p["voltage"], opto_enabled=p["opto_enabled"])

            with _lock:
                _state.update({
                    "pirani_adc":      p["adc"],
                    "pirani_voltage":  p["voltage"],
                    "opto_enabled":    p["opto_enabled"],
                    "mfc_adc":         m["adc"],
                    "mfc_voltage":     m["voltage"],
                    "mfc_flow":        m["flow"],
                    "mfc_valve_closed": m.get("valve_closed", False),
                    "dac_ready":       m.get("dac_ready", False),
                    "dac_voltage": m.get("dac_voltage", 0.0),
                    "dac_code": m.get("dac_code", 0),
                    "flow_target":     m.get("flow_target", 0.0),
                    "sm_state":        sm.state,
                    "error":           "",
                })

        except Exception as exc:
            with _lock:
                _state["error"] = str(exc)

        time.sleep(POLLING_INTERVAL)


threading.Thread(target=_poll, daemon=True).start()


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
    command=lambda: sm.transition("READY")
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

btn_flush.grid(row=0, column=0, padx=4, pady=4)
btn_sputter.grid(row=0, column=1, padx=4, pady=4)
btn_stop .grid(row=1, column=0, padx=4, pady=4)
btn_vent .grid(row=1, column=1, padx=4, pady=4)
btn_estop.grid(row=2, column=0, columnspan=2, padx=4, pady=(4, 0), sticky="ew")

# ── Pirani frame ──────────────────────────────────────
pf = tk.LabelFrame(root, text=" Pirani Gauge  [A0] ", padx=10, pady=6)
pf.grid(row=3, column=0, padx=12, pady=(6, 6), sticky="ew")

lbl_p_adc      = tk.Label(pf, text="ADC     : ——",      font=("Courier", 12), anchor="w", width=32)
lbl_p_volt     = tk.Label(pf, text="Voltage : ——.—— V", font=("Courier", 12), anchor="w", width=32)
lbl_opto       = tk.Label(pf, text="OPTO    : ——",      font=("Courier", 14, "bold"), anchor="w", width=32)
lbl_p_message  = tk.Label(pf, text="",               font=("Courier", 10), anchor="w", width=52, fg="blue")
lbl_p_adc .grid(row=0, sticky="w")
lbl_p_volt.grid(row=1, sticky="w")
lbl_opto  .grid(row=2, sticky="w", pady=(6, 0))
lbl_p_message.grid(row=3, sticky="w", pady=(6, 0))

p_canvas = tk.Canvas(pf, height=90, bg="#1a1a1a", highlightthickness=0)
p_canvas.grid(row=4, column=0, sticky="ew", pady=(6, 2))
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
lbl_m_argon  = tk.Label(mff, text="Argon PSI:",            font=("Courier", 12), anchor="w")
entry_argon  = tk.Entry(mff, width=10, font=("Courier", 12))
btn_argon    = tk.Button(mff, text="Update", font=("Courier", 10), command=lambda: _update_argon_pressure())

lbl_m_adc   .grid(row=0, sticky="w")
lbl_m_volt  .grid(row=1, sticky="w")
lbl_m_flow  .grid(row=2, sticky="w", pady=(6, 0))
lbl_m_dac   .grid(row=3, sticky="w", pady=(6, 0))
lbl_m_valve .grid(row=4, sticky="w", pady=(6, 0))

lbl_m_argon.grid(row=5, column=0, sticky="w", pady=(6, 0))
entry_argon.grid(row=5, column=1, sticky="w", pady=(6, 0))
btn_argon.grid(row=5, column=2, padx=(6, 0), pady=(6, 0))

m_canvas = tk.Canvas(mff, height=90, bg="#1a1a1a", highlightthickness=0)
m_canvas.grid(row=6, column=0, columnspan=3, sticky="ew", pady=(6, 2))
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
BUTTON_STATES = {
    "IDLE":            ("disabled", "disabled", "disabled", "disabled", "normal"),
    "PUMP_DOWN":       ("disabled", "disabled", "disabled", "disabled", "normal"),
    "READY":           ("normal",   "disabled", "disabled", "normal",   "normal"),
    "ARGON_FLUSH":     ("disabled", "disabled", "disabled", "normal",   "normal"),
    "PLASMA_IGNITING": ("disabled", "disabled", "normal",   "normal",   "normal"),
    "SPUTTER_READY":   ("disabled", "normal",   "normal",   "normal",   "normal"),
    "SPUTTERING":      ("disabled", "disabled", "normal",   "normal",   "normal"),
    "VENTING":         ("disabled", "disabled", "disabled", "disabled", "normal"),
}


# ════════════════════════════════════════════════════════
#  GUI REFRESH
# ════════════════════════════════════════════════════════

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


def _ignite_plasma():
    """Trigger plasma ignition via control signal.

    Integrates with RF power supply, pressure feedback, and safety interlocks.
    Placeholder for hardware control implementation.
    """
    # TODO: implement RF trigger/control logic when hardware is defined
    pass


def _refresh():
    with _lock:
        s = dict(_state)

    # — State —
    st = s["sm_state"]
    lbl_state.config(text=st, fg=STATE_COLORS.get(st, "grey"))

    # — Buttons —
    bs = BUTTON_STATES.get(st, BUTTON_STATES["IDLE"])
    btn_flush  .config(state=bs[0])
    btn_sputter.config(state=bs[1])
    btn_stop   .config(state=bs[2])
    btn_vent   .config(state=bs[3])
    btn_estop  .config(state=bs[4])

    # — Pirani —
    lbl_p_adc .config(text=f"ADC     : {s['pirani_adc']}")
    lbl_p_volt.config(text=f"Voltage : {s['pirani_voltage']:.6f} V")
    lbl_opto  .config(
        text = "OPTO    : ON " if s["opto_enabled"] else "OPTO    : OFF",
        fg   = "green"         if s["opto_enabled"] else "red",
    )

    if st == "IDLE":
        p_message = "IDLE: MFC valve closed, turbo opto off."
    elif st == "PUMP_DOWN":
        if s["pirani_voltage"] <= IDLE_PRESSURE_MAX_VOLTAGE and not s["opto_enabled"]:
            p_message = "Pressure below 10 mbar. Turbo can now be started; turbo opto has been enabled."
        elif s["pirani_voltage"] <= IDLE_PRESSURE_MAX_VOLTAGE and s["opto_enabled"]:
            p_message = "Turbo opto is on; pressure should continue to drop toward zero."
        else:
            p_message = "Pump down continues; wait until pressure falls below 10 mbar before turbo start."
    elif st == "READY":
        p_message = "READY: Start Argon Flush once inlet pressure is ≥ 15 psi."
    elif st == "ARGON_FLUSH":
        if s["pirani_voltage"] >= ARGON_FLUSH_TARGET_VOLTAGE:
            p_message = "Argon flush complete; igniting plasma automatically."
        else:
            p_message = "Argon flush active: increasing MFC pressure to reach 0.1 mbar."
    elif st == "PLASMA_IGNITING":
        p_message = "Plasma ignition in progress; waiting for sputter-ready voltage ≤ 0.466 V."
    elif st == "SPUTTER_READY":
        p_message = "SPUTTER READY: Plasma ignited and Pirani voltage is ≤ 0.466 V."
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
    lbl_status.config(
        text = f"ERR: {s['error']}" if s["error"] else "OK",
        fg   = "red"                if s["error"] else "grey",
    )

    root.after(GUI_REFRESH_INTERVAL, _refresh)


# ════════════════════════════════════════════════════════
#  CLEAN SHUTDOWN
# ════════════════════════════════════════════════════════
def _on_close():
    pirani.shutdown()
    GPIO.cleanup()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", _on_close)

_refresh()
root.mainloop()
