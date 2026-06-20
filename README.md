# Sputter Vacuum Controller

A Raspberry Pi-based vacuum process controller for DC magnetron sputter deposition. Manages chamber pump-down, argon gas flow, plasma ignition, and sputtering via a live Tkinter GUI with scrolling sensor graphs and a hardware-interlocked state machine.

---

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [Software Architecture](#software-architecture)
- [File Structure](#file-structure)
- [State Machine](#state-machine)
- [Pressure Reference](#pressure-reference)
- [Installation](#installation)
- [Running](#running)
- [Configuration](#configuration)
- [GUI Guide](#gui-guide)
- [Known Limitations](#known-limitations)
- [Pending Features](#pending-features)

---

## Hardware Overview

| Component | Model | Interface | Notes |
|---|---|---|---|
| Microcontroller | Raspberry Pi (BCM GPIO) | — | Host for all control logic |
| Pirani gauge | ACE Instruments DHPG-015 | Analog 0–10V | Divided to 0–3.3V before ADC |
| ADC | ADS1115 | I2C @ 0x48 | Pirani on A0, MFC feedback on A1 |
| MFC | MKS 1179A | Analog 0–5V setpoint | 0–700 sccm Argon |
| DAC | MCP4725 | I2C @ 0x60 | Powered off Pi 3.3V rail — no level shifter fitted |
| Turbo interlock | Opto-isolator | GPIO 17 (BCM) | Drives turbo pump enable signal |
| MFC valve close | Emergency shut | GPIO 27 (BCM) | Pulls MFC valve closed on demand |

### Voltage Divider (Pirani Output)

The DHPG-015 outputs 0–10V. A 3-resistor voltage divider scales this to 0–3.3V for the ADS1115. All pressure thresholds in `config.py` are expressed in post-divider volts (multiply by 3.03 to get raw gauge voltage).

### DAC Limitation

The MCP4725 DAC is powered from the Pi's 3.3V rail. Without a level shifter, it can only output 0–3.3V instead of the MFC's required 0–5V setpoint range. This caps commanded flow at approximately 46% of full scale (~322 sccm). A level-shift or external 5V supply circuit is required to access the full 700 sccm range.

---

## Software Architecture

```
main.py
├── Hardware init (GPIO, I2C, ADS1115)
├── Shared state dict (thread-safe, _lock)
├── Polling thread (_poll) — runs every 0.5s
│   ├── Reads Pirani + MFC sensors
│   ├── Drives MFC pressure P-loop
│   ├── Calls sm.update() for auto-transitions
│   └── Updates shared state dict
├── Tkinter GUI (main thread)
│   ├── _refresh() — runs every 500ms via root.after()
│   ├── Scrolling graphs (ScrollingGraph)
│   └── Button callbacks → sm.transition()
└── Hardware interlock callback
    └── handle_hardware_interlocks() — fires on every state transition
```

The polling thread and GUI thread never share objects directly — all data passes through `_state` dict protected by `_lock`.

---

## File Structure

```
Sputter_ctrl/
├── main.py           — Entry point: GUI, polling thread, hardware init
├── config.py         — All constants, thresholds, and pin assignments
├── state_machine.py  — SputterStateMachine class
├── mfc_control.py    — MFCController class (DAC, valve, P-loop)
├── pirani.py         — PiraniController class (ADC, opto)
├── graph.py          — ScrollingGraph class (pure Tkinter, no matplotlib)
└── README.md
```

---

## State Machine

### States and Transitions

```
                    pressure rises
         ┌─────────────────────────────┐
         ▼                             │
       IDLE ──(pressure drops)──► PUMP_DOWN ──(vacuum reached + opto)──► READY
         ▲                             │                                     │
         │                    pressure rises                        [Start Argon Flush]
         │                             │                                     ▼
         └─────────────────────────────┘                            ARGON_FLUSH
                                                                         │
                                                              (0.09 mbar reached)
                                                                         ▼
                                                               PLASMA_IGNITING
                                                              │              │
                                                   [Confirm Plasma]    (2 min timeout)
                                                              │              │
                                                              ▼              ▼
                                                        SPUTTER_READY     READY
                                                              │
                                                       [Start Sputter]
                                                              ▼
                                                          SPUTTERING
                                                              │
                                                       [Stop Sputter]
                                                              ▼
                                                            READY

         Any state ──[Vent]──► VENTING ──(atmosphere)──► IDLE
         Any state ──[E-STOP]──► IDLE (immediate hardware cutoff)
```

### Automatic Transitions

| From | To | Condition | Where |
|---|---|---|---|
| `IDLE` | `PUMP_DOWN` | Pirani voltage ≤ 2.71V (10 mbar) | `_poll()` |
| `PUMP_DOWN` | `IDLE` | Pirani voltage ≥ 2.71V (pump failure/leak) | `_poll()` + `sm.update()` |
| `PUMP_DOWN` | `READY` | Voltage ≤ 0.051V AND opto enabled | `sm.update()` |
| `READY` | `PUMP_DOWN` | Pirani voltage ≥ 2.71V (pressure degraded) | `sm.update()` |
| `ARGON_FLUSH` | `PLASMA_IGNITING` | Voltage ≥ 1.287V (0.09 mbar) | `_poll()` + `sm.update()` |
| `PLASMA_IGNITING` | `READY` | 120s timeout, no plasma confirmation | `_poll()` |
| `VENTING` | `IDLE` | ADC counts ≥ 30000 (atmosphere) | `sm.update()` |

### Manual Transitions (Operator Buttons)

| Button | Transition | Guard |
|---|---|---|
| Start Argon Flush | `READY → ARGON_FLUSH` | Argon inlet ≥ 15 psi |
| Confirm Plasma | `PLASMA_IGNITING → SPUTTER_READY` | None |
| Start Sputter | `SPUTTER_READY → SPUTTERING` | Argon inlet ≥ 15 psi |
| Stop Sputter | `SPUTTERING → READY` | None |
| Vent | Most states → `VENTING` | None |
| E-STOP | Any → `IDLE` | None — immediate |

### Hardware Interlock Callback

`handle_hardware_interlocks()` fires synchronously on every state transition:

- **→ IDLE or VENTING**: `mfc.set_flow(0)`, `mfc.valve_close()`, `pirani.set_opto(False)`
- **→ PUMP_DOWN**: `pirani.set_opto(False)` — resets opto for next pump-down cycle

---

## Pressure Reference

All voltages are post-divider (ADC input). Raw gauge output = voltage × 3.03.

| Pressure (mbar) | Raw gauge (V) | ADC input (V) | Significance |
|---|---|---|---|
| 999 | 10.00 | 3.30 | Atmosphere (ADC saturated) |
| 10 | 8.20 | 2.706 | `IDLE_PRESSURE_MAX_VOLTAGE` — pump-down gate |
| 0.09 | 3.90 | 1.287 | `ARGON_FLUSH_TARGET_VOLTAGE` — plasma ignition pressure |
| 0.007 | 1.10 | 0.363 | `SPUTTER_READY_TARGET_VOLTAGE` — sputtering pressure |
| 0.002 | 0.35 | 0.116 | `PUMP_DOWN_COMPLETE_VOLTAGE` — high vacuum confirmed |
| 0.001 | 0.12 | 0.040 | Over-range threshold |

### Opto Logic

The turbo pump interlock (GPIO 17) uses **transition-based** logic, not level-based:

- Goes **HIGH** the first time Pirani voltage drops below **1.2V** during `PUMP_DOWN`
- Once set, stays HIGH until any state transition resets it via the interlock callback
- This prevents opto chatter on noisy ADC readings near the threshold

---

## Installation

### Dependencies

```bash
pip install adafruit-blinka adafruit-circuitpython-ads1x15 RPi.GPIO --break-system-packages
```

### Enable I2C on the Pi

```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

Verify devices are visible:

```bash
i2cdetect -y 1
# Should show 0x48 (ADS1115) and 0x60 (MCP4725)
```

### Transfer Files

```bash
scp -O main.py config.py mfc_control.py state_machine.py pirani.py graph.py \
    raspberrypi@<pi-ip>:~/Sputter_ctrl/
```

---

## Running

Must be run on the Pi with a display connected (HDMI or VNC), not over a plain SSH session.

```bash
cd ~/Sputter_ctrl
python3 main.py
```

To run over SSH with display forwarding:

```bash
ssh -X raspberrypi@<pi-ip>
cd ~/Sputter_ctrl && python3 main.py
```

---

## Configuration

All tunable parameters are in `config.py`. Key constants:

```python
# Pressure thresholds (post-divider volts)
IDLE_PRESSURE_MAX_VOLTAGE    = 2.71    # 10 mbar  — pump-down gate
PUMP_DOWN_COMPLETE_VOLTAGE   = 0.051   # 0.002 mbar — high vacuum confirmed
ARGON_FLUSH_TARGET_VOLTAGE   = 1.287   # 0.09 mbar — plasma ignition pressure
SPUTTER_READY_TARGET_VOLTAGE = 0.363   # 0.007 mbar — sputtering pressure

# Pressure control
ARGON_FLUSH_FLOW_SETPOINT    = 150.0   # sccm — initial flow on flush entry
PRESSURE_CONTROL_KP          = 40.0    # sccm/V — proportional gain; tune if needed

# Timing
PLASMA_IGNITION_TIMEOUT      = 120.0   # seconds — auto-abort if plasma not confirmed
POLLING_INTERVAL             = 0.5     # seconds between sensor reads
GUI_REFRESH_INTERVAL         = 500     # ms between GUI updates

# Hardware
GPIO_PIRANI_PIN              = 17      # BCM — turbo opto
GPIO_MFC_VALVE_CLOSE_PIN     = 27      # BCM — emergency valve close
ADS1115_I2C_ADDRESS          = 0x48
ARGON_DAC_I2C_ADDRESS        = 0x60
MFC_FULL_SCALE               = 700.0   # sccm
MFC_GAS_CORRECTION_FACTOR    = 1.39    # Argon correction for MKS 1179A
```

### Tuning the Pressure P-Loop

If the chamber pressure oscillates around the target, reduce `PRESSURE_CONTROL_KP`. If it converges too slowly, increase it. A steady-state offset (pressure settles near but not at target) indicates an I-term is needed — add it to `pressure_control_step()` in `mfc_control.py`.

---

## GUI Guide

| Element | Description |
|---|---|
| **Process State** | Current state, colour-coded |
| **Start Argon Flush** | Opens MFC valve, starts P-loop to 0.09 mbar |
| **Confirm Plasma** | Manually confirms plasma ignition; transitions to SPUTTER_READY |
| **Start Sputter** | Begins sputtering from SPUTTER_READY |
| **Stop Sputter** | Returns to READY with vacuum maintained |
| **Vent** | Vents chamber to atmosphere |
| **E-STOP** | Immediate: kills flow, closes valve, drops opto, returns to IDLE |
| **Argon PSI** | Enter upstream argon regulator pressure; required ≥ 15 psi to start flush/sputter |
| **Pirani graph** | Scrolling ADC voltage (lime, 0–4V, 60 samples) |
| **MFC graph** | Scrolling flow sccm (cyan) overlaid with DAC voltage (yellow dashed) |
| **OPTO** | Green = turbo interlock enabled; Red = off |
| **DAC OUT** | Green = DAC found on I2C; Red = not detected |
| **Valve** | Green = released (MFC in control); Red = closed (emergency) |

---

## Known Limitations

- **DAC ceiling at ~3.3V**: MCP4725 powered from Pi 3.3V rail limits MFC setpoint to ~46% of full scale. A level shifter or dedicated 5V DAC supply is required for full flow range.
- **`_ignite_plasma()` is a stub**: RF power supply trigger is not yet implemented. The function exists as a placeholder for hardware integration.
- **SPUTTERING state has no P-loop**: Pressure is not actively controlled during sputtering. This is intentional pending further process definition.
- **No I-term in pressure control**: Pure proportional control may exhibit steady-state offset at low pressures. Add integral term if needed.

---

## Pending Features

- [ ] RF power supply trigger in `_ignite_plasma()`
- [ ] Current sensor integration for automatic plasma detection
- [ ] Plasma frame GUI controls (right panel — currently placeholder)
- [ ] DAC level shifter for full MFC range
- [ ] Pressure P+I loop if steady-state offset is observed
- [ ] SPUTTERING state pressure maintenance
- [ ] Datalog / CSV export of Pirani and MFC readings
