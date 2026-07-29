
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drivers.pzem_meter import PZEMController

# -----------------------------------
# Bench script: live PZEM-004T-100A readings over the CP2102 USB-TTL
# adapter. Run with main.py stopped -- they'd otherwise fight over the
# same serial port.
# -----------------------------------

pzem = PZEMController()

while True:
    r = pzem.read()
    print("--------------------------------")
    if not r["ready"]:
        print(f"NOT READY ({r.get('error')})")
    else:
        print(f"Voltage  : {r['voltage']:.1f} V")
        print(f"Current  : {r['current']:.3f} A")
        print(f"Power    : {r['power']:.1f} W")
        print(f"Energy   : {r['energy']:.0f} Wh")
        print(f"Frequency: {r['frequency']:.1f} Hz")
        print(f"PF       : {r['power_factor']:.2f}")
        print(f"Alarm    : {r['alarm']}")
    time.sleep(0.5)
