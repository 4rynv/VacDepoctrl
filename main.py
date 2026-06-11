#!/usr/bin/env python3
"""
main.py — Sputter Vacuum Controller
State machine + live scrolling graphs for Pirani voltage and MFC flow.
"""

import socket
import threading
import time
import tkinter as tk

import board
import busio
import RPi.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS

from pirani        import PiraniController
from mfc_control   import MFCController
from state_machine import SputterStateMachine, STATE_COLORS
from graph         import ScrollingGraph


# ════════════════════════════════════════════════════════
#  HARDWARE INIT
# ════════════════════════════════════════════════════════
GPIO.setmode(GPIO.BCM)

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
ads.gain = 1

pirani = PiraniController(ads, opto_pin=17)
mfc    = MFCController(ads)
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
    "pirani_adc":     0,
    "pirani_voltage": 0.0,
    "opto_enabled":   False,
    "mfc_adc":        0,
    "mfc_voltage":    0.0,
    "mfc_flow":       0.0,
    "sm_state":       "IDLE",
    "error":          "",
}


# ════════════════════════════════════════════════════════
#  POLLING THREAD
# ════════════════════════════════════════════════════════
def _poll():
    while True:
        try:
            current = sm.state

            if current in ("IDLE", "VENTING"):
                p = pirani.read(auto_opto=False)
                pirani.set_opto(False)
            else:
                p = pirani.read(auto_opto=True)

            m = mfc.read()
            sm.update(p["adc"])

            with _lock:
                _state.update({
                    "pirani_adc":     p["adc"],
                    "pirani_voltage": p["voltage"],
                    "opto_enabled":   p["opto_enabled"],
                    "mfc_adc":        m["adc"],
                    "mfc_voltage":    m["voltage"],
                    "mfc_flow":       m["flow"],
                    "sm_state":       sm.state,
                    "error":          "",
                })

        except Exception as exc:
            with _lock:
                _state["error"] = str(exc)

        time.sleep(0.5)


threading.Thread(target=_poll, daemon=True).start()


# ════════════════════════════════════════════════════════
#  TKINTER GUI
# ════════════════════════════════════════════════════════
root = tk.Tk()
root.title("Sputter Vacuum Controller")
root.resizable(False, False)

# ── IP bar ───────────────────────────────────────────
ip_frame = tk.Frame(root)
ip_frame.grid(row=0, column=0, padx=12, pady=(10, 0), sticky="ew")
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

btn_pump  = tk.Button(bf, text="Start Pump Down", width=16, font=("Courier", 10),
                      command=lambda: sm.transition("PUMP_DOWN"))
btn_start = tk.Button(bf, text="Start Sputter",   width=16, font=("Courier", 10),
                      command=lambda: sm.transition("SPUTTERING"))
btn_stop  = tk.Button(bf, text="Stop Sputter",    width=16, font=("Courier", 10),
                      command=lambda: sm.transition("READY"))
btn_vent  = tk.Button(bf, text="Vent",            width=16, font=("Courier", 10),
                      command=lambda: sm.transition("VENTING"))
btn_estop = tk.Button(bf, text="E-STOP",          width=16, font=("Courier", 10, "bold"),
                      fg="white", bg="red",
                      command=sm.emergency_stop)

btn_pump .grid(row=0, column=0, padx=4, pady=4)
btn_start.grid(row=0, column=1, padx=4, pady=4)
btn_stop .grid(row=1, column=0, padx=4, pady=4)
btn_vent .grid(row=1, column=1, padx=4, pady=4)
btn_estop.grid(row=2, column=0, columnspan=2, padx=4, pady=(4, 0), sticky="ew")

# ── Pirani frame ──────────────────────────────────────
pf = tk.LabelFrame(root, text=" Pirani Gauge  [A0] ", padx=10, pady=6)
pf.grid(row=3, column=0, padx=12, pady=(6, 6), sticky="ew")

lbl_p_adc  = tk.Label(pf, text="ADC     : ——",      font=("Courier", 12), anchor="w", width=32)
lbl_p_volt = tk.Label(pf, text="Voltage : ——.—— V", font=("Courier", 12), anchor="w", width=32)
lbl_opto   = tk.Label(pf, text="OPTO    : ——",      font=("Courier", 14, "bold"), anchor="w", width=32)
lbl_p_adc .grid(row=0, sticky="w")
lbl_p_volt.grid(row=1, sticky="w")
lbl_opto  .grid(row=2, sticky="w", pady=(6, 0))

p_canvas = tk.Canvas(pf, height=90, bg="#1a1a1a", highlightthickness=0)
p_canvas.grid(row=3, column=0, sticky="ew", pady=(6, 2))
pf.columnconfigure(0, weight=1)

p_graph = ScrollingGraph(p_canvas, maxlen=60,
                         min_val=0.0, max_val=4.0,
                         line_color="lime", unit="V")

# ── MFC frame ─────────────────────────────────────────
mff = tk.LabelFrame(root, text=" MFC  Argon Flow  [A1] ", padx=10, pady=6)
mff.grid(row=4, column=0, padx=12, pady=(0, 6), sticky="ew")

lbl_m_adc  = tk.Label(mff, text="ADC     : ——",            font=("Courier", 12), anchor="w", width=32)
lbl_m_volt = tk.Label(mff, text="Voltage : ——.—— V",       font=("Courier", 12), anchor="w", width=32)
lbl_m_flow = tk.Label(mff, text="Flow    : ——.—— sccm Ar", font=("Courier", 14, "bold"), anchor="w", width=32)
lbl_m_adc .grid(row=0, sticky="w")
lbl_m_volt.grid(row=1, sticky="w")
lbl_m_flow.grid(row=2, sticky="w", pady=(6, 0))

m_canvas = tk.Canvas(mff, height=90, bg="#1a1a1a", highlightthickness=0)
m_canvas.grid(row=3, column=0, sticky="ew", pady=(6, 2))
mff.columnconfigure(0, weight=1)

m_graph = ScrollingGraph(m_canvas, maxlen=60,
                         min_val=0.0, max_val=700.0,
                         line_color="cyan", unit="sccm")

# ── Status bar ────────────────────────────────────────
lbl_status = tk.Label(root, text="OK", font=("Courier", 10),
                      anchor="w", width=32, fg="grey")
lbl_status.grid(row=5, column=0, padx=12, pady=(0, 10), sticky="w")


# ════════════════════════════════════════════════════════
#  BUTTON STATE MAP
# ════════════════════════════════════════════════════════
BUTTON_STATES = {
    "IDLE":       ("normal",   "disabled", "disabled", "disabled", "disabled"),
    "PUMP_DOWN":  ("disabled", "disabled", "disabled", "disabled", "normal"),
    "READY":      ("disabled", "normal",   "disabled", "normal",   "normal"),
    "SPUTTERING": ("disabled", "disabled", "normal",   "disabled", "normal"),
    "VENTING":    ("disabled", "disabled", "disabled", "disabled", "normal"),
}


# ════════════════════════════════════════════════════════
#  GUI REFRESH
# ════════════════════════════════════════════════════════
def _refresh():
    with _lock:
        s = dict(_state)

    # — State —
    st = s["sm_state"]
    lbl_state.config(text=st, fg=STATE_COLORS.get(st, "grey"))

    # — Buttons —
    bs = BUTTON_STATES.get(st, BUTTON_STATES["IDLE"])
    btn_pump .config(state=bs[0])
    btn_start.config(state=bs[1])
    btn_stop .config(state=bs[2])
    btn_vent .config(state=bs[3])
    btn_estop.config(state=bs[4])

    # — Pirani —
    lbl_p_adc .config(text=f"ADC     : {s['pirani_adc']}")
    lbl_p_volt.config(text=f"Voltage : {s['pirani_voltage']:.6f} V")
    lbl_opto  .config(
        text = "OPTO    : ON " if s["opto_enabled"] else "OPTO    : OFF",
        fg   = "green"         if s["opto_enabled"] else "red",
    )
    p_graph.push(s["pirani_voltage"])
    p_graph.draw()

    # — MFC —
    lbl_m_adc .config(text=f"ADC     : {s['mfc_adc']}")
    lbl_m_volt.config(text=f"Voltage : {s['mfc_voltage']:.6f} V")
    lbl_m_flow.config(text=f"Flow    : {s['mfc_flow']:.2f} sccm Ar")
    m_graph.push(s["mfc_flow"])
    m_graph.draw()

    # — Status —
    lbl_status.config(
        text = f"ERR: {s['error']}" if s["error"] else "OK",
        fg   = "red"                if s["error"] else "grey",
    )

    root.after(500, _refresh)


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
