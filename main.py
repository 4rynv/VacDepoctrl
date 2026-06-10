#!/usr/bin/env python3
"""
main.py — Sputter Vacuum Controller
Imports PiraniController and MFCController, shares one ADS1115 between them.
"""

import socket
import threading
import time
import tkinter as tk
import board
import busio
import RPi.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS

from pirani      import PiraniController
from mfc_control import MFCController


# ════════════════════════════════════════════════════════
#  HARDWARE INIT  (shared — must happen before imports use GPIO)
# ════════════════════════════════════════════════════════
GPIO.setmode(GPIO.BCM)

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
ads.gain = 1

# ════════════════════════════════════════════════════════
#  CONTROLLERS  (both receive the same ads object)
# ════════════════════════════════════════════════════════
pirani = PiraniController(ads, opto_pin=17)
mfc    = MFCController(ads)


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
    "error":          "",
}


# ════════════════════════════════════════════════════════
#  POLLING THREAD  — hardware only, never touches widgets
# ════════════════════════════════════════════════════════
def _poll():
    while True:
        try:
            p = pirani.read()
            m = mfc.read()

            with _lock:
                _state.update({
                    "pirani_adc":     p["adc"],
                    "pirani_voltage": p["voltage"],
                    "opto_enabled":   p["opto_enabled"],
                    "mfc_adc":        m["adc"],
                    "mfc_voltage":    m["voltage"],
                    "mfc_flow":       m["flow"],
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

# ── IP Address bar ───────────────────────────────────
ip_frame = tk.Frame(root)
ip_frame.grid(row=0, column=0, padx=12, pady=(10, 0), sticky="ew")
tk.Label(ip_frame, text="Pi IP :", font=("Courier", 11, "bold"), anchor="w").pack(side="left")
lbl_ip = tk.Label(ip_frame, text=_get_ip(), font=("Courier", 11), fg="blue", anchor="w")
lbl_ip.pack(side="left")

# ── Pirani Frame ─────────────────────────────────────
pf = tk.LabelFrame(root, text=" Pirani Gauge  [A0] ", padx=10, pady=6)
pf.grid(row=1, column=0, padx=12, pady=(10, 6), sticky="ew")

lbl_p_adc  = tk.Label(pf, text="ADC     : ——",      font=("Courier", 12), anchor="w", width=32)
lbl_p_volt = tk.Label(pf, text="Voltage : ——.—— V", font=("Courier", 12), anchor="w", width=32)
lbl_opto   = tk.Label(pf, text="OPTO    : ——",      font=("Courier", 14, "bold"), anchor="w", width=32)
lbl_p_adc .grid(row=0, sticky="w")
lbl_p_volt.grid(row=1, sticky="w")
lbl_opto  .grid(row=2, sticky="w", pady=(6, 0))

# ── MFC Frame ────────────────────────────────────────
mff = tk.LabelFrame(root, text=" MFC  Argon Flow  [A1] ", padx=10, pady=6)
mff.grid(row=2, column=0, padx=12, pady=(6, 6), sticky="ew")

lbl_m_adc  = tk.Label(mff, text="ADC     : ——",            font=("Courier", 12), anchor="w", width=32)
lbl_m_volt = tk.Label(mff, text="Voltage : ——.—— V",       font=("Courier", 12), anchor="w", width=32)
lbl_m_flow = tk.Label(mff, text="Flow    : ——.—— sccm Ar", font=("Courier", 14, "bold"), anchor="w", width=32)
lbl_m_adc .grid(row=0, sticky="w")
lbl_m_volt.grid(row=1, sticky="w")
lbl_m_flow.grid(row=2, sticky="w", pady=(6, 0))

# ── Status bar ───────────────────────────────────────
lbl_status = tk.Label(root, text="OK", font=("Courier", 10),
                      anchor="w", width=32, fg="grey")
lbl_status.grid(row=3, column=0, padx=12, pady=(0, 10), sticky="w")


# ════════════════════════════════════════════════════════
#  GUI REFRESH  — runs on main thread via after()
# ════════════════════════════════════════════════════════
def _refresh():
    with _lock:
        s = dict(_state)

    lbl_p_adc .config(text=f"ADC     : {s['pirani_adc']}")
    lbl_p_volt.config(text=f"Voltage : {s['pirani_voltage']:.6f} V")
    lbl_opto  .config(
        text = "OPTO    : ON " if s["opto_enabled"] else "OPTO    : OFF",
        fg   = "green"         if s["opto_enabled"] else "red",
    )

    lbl_m_adc .config(text=f"ADC     : {s['mfc_adc']}")
    lbl_m_volt.config(text=f"Voltage : {s['mfc_voltage']:.6f} V")
    lbl_m_flow.config(text=f"Flow    : {s['mfc_flow']:.2f} sccm Ar")

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
