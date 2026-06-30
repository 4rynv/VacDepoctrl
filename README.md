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
| Microcontroller | Raspberry Pi 2B (BCM GPIO) | — | Host for all control logic |
| Pirani gauge | ACE Instruments DHPG-015 | Analog 0–10V | Divided to 0–3.3V before ADC (ratio 0.33) |
| ADC | ADS1115 | I2C @ 0x48 | Pirani on A0, MFC feedback on A1 |
| MFC | MKS 1179A | Analog 0–5V setpoint | 0–700 sccm Argon |
| DAC | MCP4725 | I2C @ 0x60 | Powered off Pi 3.3V rail — buffered, see below |
| DAC buffer | LM358P op-amp | Analog | Unity-gain follower; required, DAC alone cannot drive MFC setpoint input |
| Turbo interlock | Opto-isolator (4N35) | GPIO 17 (BCM) | Drives turbo pump enable signal |
| MFC valve close | Emergency shut | GPIO 27 (BCM) | Pulls MFC valve closed on demand |
| Turbo inlet valve opto | Opto-isolator (4N35) | GPIO 4 (BCM) | HIGH = valve closed, LOW = valve open |

### Voltage Divider (Pirani Output)

The DHPG-015 outputs 0–10V. A 3-resistor voltage divider scales this to 0–3.3V for the ADS1115 (divider ratio 0.33). All pressure thresholds in `config.py` are expressed in post-divider (ADC-side) volts.

### DAC Output Buffer (LM358P)

The MCP4725 cannot source enough current to drive the MFC setpoint input directly — above ~1.25V the output voltage collapses under load even though it reads correctly when unloaded. An LM358P wired as a unity-gain voltage follower buffers the DAC output before it reaches the MFC:

```
DAC OUT ──────────────┬──── LM358 IN+ (pin 3)
                       │
                  (feedback) IN- (pin 2) ──── OUT (pin 1) ──── MFC setpoint
LM358 VCC (pin 8) ── Pi 5V
LM358 GND (pin 4) ── GND
```

Power the op-amp from the Pi's **5V** rail, not 3.3V — at 3.3V supply the LM358's output headroom (~1.5V below V+) caps it at ~1.8V, which doesn't improve on the unbuffered DAC ceiling. At 5V supply the usable output range comfortably covers the full 0–3.3V DAC range.

`ARGON_DAC_VREF` in `config.py` is set to `3.3` (not 5.0) since this is the real ceiling of the MCP4725's own output stage — the buffer doesn't increase available *voltage*, only current.

### Turbo Inlet Valve Interlock (GPIO 4)

A second opto-isolator gates the turbo pump inlet valve, independent of the turbo enable opto (GPIO 17):

- Valve is **open** when the opto is **off**, **closed** when the opto is **on**
- Closed any time the turbo motor is running
- Currently gated on chamber pressure (no turbo RPM readout wired up yet): valve opens automatically once pressure rises above **0.1 mbar**
- A proper RPM-based interlock (EXC120 analogue speed output, pins 16/17) is on the pending list — see [Pending Features](#pending-features)

### Known Opto Failure Mode

Two 4N35 opto-isolators failed during testing on the turbo enable line (GPIO 17). Root cause suspected to be inductive kickback from the turbo controller's input circuit on switch-off, exceeding the phototransistor's collector-emitter rating transiently. **Mitigation not yet installed**: a 1N4148 (or 1N4001) flyback diode across the opto's collector-emitter, plus a pulldown resistor, is recommended before further testing. See [Known Limitations](#known-limitations).

---

## Software Architecture

```
main.py
├── Hardware init (GPIO, I2C, ADS1115)
├── Shared state dict (thread-safe, _lock)
├── Polling thread (_poll) — runs every POLLING_INTERVAL (0.2s)
│   ├── Reads Pirani + MFC sensors
│   ├── Drives MFC pressure PID loop
│   ├── Calls sm.update() for auto-transitions
│   ├── Converts Pirani voltage <-> mbar via log-linear calibration table
│   └── Updates shared state dict
├── Tkinter GUI (main thread)
│   ├── _refresh() — runs every GUI_REFRESH_INTERVAL (200ms) via root.after()
│   ├── Scrolling graphs (ScrollingGraph)
│   ├── Manual set-point entry fields (sputter target mbar, argon PSI) with Enter-to-submit
│   ├── Live IP address display
│   └── Button callbacks → sm.transition()
└── Hardware interlock callback
    └── handle_hardware_interlocks() — fires on every state transition
```

The polling thread and GUI thread never share objects directly — all data passes through `_state` dict protected by `_lock`. On window close, `_stop_event` signals the polling thread to exit before `GPIO.cleanup()` runs, avoiding a race that previously caused `[HW ERROR] Polling loop glitch` on shutdown.

---

## File Structure

```
Sputter_ctrl/
├── main.py           — Entry point: GUI, polling thread, hardware init
├── config.py         — All constants, thresholds, and pin assignments
├── state_machine.py  — SputterStateMachine class
├── mfc_control.py    — MFCController class (DAC, valve, PID loop)
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
                                                          VENTING
                                                              │
                                                      (2.5V atmosphere)
                                                              ▼
                                                            IDLE

         Any state ──[Vent]──► VENTING ──(2.5V Pirani reading)──► IDLE
         Any state ──[E-STOP]──► IDLE (immediate hardware cutoff)
```

**Changed this session:** Stop Sputter now transitions directly to `VENTING` instead of `READY` — sputtering should always be followed by a vent cycle rather than holding vacuum. The Start Sputter / Stop Sputter buttons now occupy the same grid cell and swap visibility based on state, rather than both being shown with one disabled.

### Automatic Transitions

| From | To | Condition | Where |
|---|---|---|---|
| `IDLE` | `PUMP_DOWN` | Pirani voltage ≤ 2.71V (10 mbar) | `_poll()` |
| `PUMP_DOWN` | `IDLE` | Pirani voltage ≥ 2.71V (pump failure/leak) | `_poll()` + `sm.update()` |
| `PUMP_DOWN` | `READY` | Voltage ≤ 0.15V AND opto enabled | `sm.update()` |
| `READY` | `PUMP_DOWN` | Pirani voltage ≥ 2.71V (pressure degraded) | `sm.update()` |
| `ARGON_FLUSH` | `PLASMA_IGNITING` | Voltage ≥ 1.287V (0.09 mbar) | `_poll()` only — guarded against double-fire (see below) |
| `PLASMA_IGNITING` | `READY` | 120s timeout, no plasma confirmation | `_poll()` |
| `VENTING` | `IDLE` | Pirani voltage ≥ **2.5V** | `sm.update()` — switched from ADC-count threshold to direct voltage this session |

**Double-transition fix:** Previously both `_poll()` and `sm.update()` independently checked the ARGON_FLUSH → PLASMA_IGNITING condition on the same tick, risking `_ignite_plasma()` firing twice once RF hardware is wired up. A `plasma_ignition_triggered` flag in shared state now gates this — set when `_poll()` fires the transition, cleared on confirm or timeout.

### Manual Transitions (Operator Buttons)

| Button | Transition | Guard |
|---|---|---|
| Start Argon Flush | `READY → ARGON_FLUSH` | Argon inlet ≥ 15 psi |
| Confirm Plasma | `PLASMA_IGNITING → SPUTTER_READY` | None |
| Start Sputter | `SPUTTER_READY → SPUTTERING` | Argon inlet ≥ 15 psi |
| Stop Sputter | `SPUTTERING → VENTING` | None *(changed from → READY)* |
| Vent | Most states → `VENTING` | None |
| E-STOP | Any → `IDLE` | None — immediate |

### Hardware Interlock Callback

`handle_hardware_interlocks()` fires synchronously on every state transition:

- **→ IDLE or VENTING**: `mfc.set_flow(0)`, `mfc.valve_close()`, `pirani.set_opto(False)`, resets `sputter_target_mbar` and `argon_pressure` in shared state back to their defaults (0.007 mbar, 0 psi), and clears `_manual_dac_override`
- **→ PUMP_DOWN**: `pirani.set_opto(False)` — resets opto for next pump-down cycle

The previous version had this split across two redundant `if new_state in ["IDLE", "VENTING"]` blocks (one for MFC, one for opto) — consolidated into one this session.

---

## Pressure Reference

All voltages are post-divider (ADC input, ×0.33 from raw gauge output). Conversion between Pirani voltage and mbar now uses **log-linear interpolation** against the full manufacturer calibration table (`_PIRANI_CAL` in `main.py`), not a fixed linear scale — the Pirani gauge's thermal-conductivity response is logarithmic, so linear interpolation introduced visible error at intermediate setpoints.

| Pressure (mbar) | Raw gauge (V) | ADC input (V) | Significance |
|---|---|---|---|
| 999 | 10.00 | 3.30 | Atmosphere (ADC saturated) |
| 10 | 8.20 | 2.706 | `IDLE_PRESSURE_MAX_VOLTAGE` — pump-down gate |
| 0.09 | 3.90 | 1.287 | `ARGON_FLUSH_TARGET_VOLTAGE` — plasma ignition pressure |
| 0.007 | 1.10 | 0.363 | `SPUTTER_READY_TARGET_VOLTAGE` — default sputtering pressure |
| ~0.01 | 1.55 | 0.512 | `PUMP_DOWN_COMPLETE_VOLTAGE` (0.15V) — high vacuum confirmed, raised from 0.051V this session for reliable PUMP_DOWN → READY triggering |
| 2.5 (ADC volts) | — | 2.5 | `VENTING_COMPLETE_VOLTAGE` — VENTING → IDLE |

The GUI now displays live pressure in mbar (`adc_voltage_to_mbar()`) alongside raw Pirani voltage, computed via the inverse of the same log-linear table.

### Sputter Target Pressure (operator-settable)

A new input field lets the operator set the SPUTTER_READY / SPUTTERING target pressure directly in mbar (default 0.007), rather than relying on the fixed `SPUTTER_READY_TARGET_VOLTAGE` constant. Entered values are converted to an ADC voltage target via the calibration table before being passed to `pressure_control_step()`. Resets to default on any transition to VENTING or IDLE.

### Opto Logic (Turbo Enable, GPIO 17)

Transition-based, not level-based:

- Goes **HIGH** the first time Pirani voltage drops below **1.2V** during `PUMP_DOWN`
- Once set, stays HIGH until any state transition resets it via the interlock callback
- This prevents opto chatter on noisy ADC readings near the threshold

---

## Installation

### 1. Clone the Repository

```bash
ssh raspberrypi@<pi-ip>
git clone https://github.com/<your-username>/Sputter_ctrl.git ~/Sputter_ctrl
cd ~/Sputter_ctrl
```

### 2. Enable I2C on the Pi

```bash
sudo raspi-config
# Interface Options → I2C → Enable
# Reboot when prompted
```

Verify both devices are visible on the I2C bus:

```bash
i2cdetect -y 1
# Should show 0x48 (ADS1115) and 0x60 (MCP4725)
```

### 3. Run the Setup Script

```bash
cd ~/Sputter_ctrl
chmod +x setup.sh
./setup.sh
```

This installs `adafruit-blinka`, `adafruit-circuitpython-ads1x15`, `adafruit-circuitpython-mcp4725`, and `RPi.GPIO` into a venv at `~/Sputter_ctrl/venv/`, and adds a `source_sputt` alias to `~/.bashrc`.

```bash
source ~/.bashrc
source_sputt
```

---

## Running

Must be run on the Pi with a display connected (HDMI or VNC), not over a plain SSH session.

```bash
source_sputt
cd ~/Sputter_ctrl
python main.py
```

> Note: use `python`, not `python3`, inside the activated venv on this setup.

To run over SSH with display forwarding:

```bash
ssh -X raspberrypi@<pi-ip>
source_sputt && cd ~/Sputter_ctrl && python main.py
```

The Pi's current IP address is shown live in the GUI header and updates automatically if the network changes.

---

## Configuration

All tunable parameters are in `config.py`. Key constants:

```python
# Pressure thresholds (post-divider volts unless noted)
IDLE_PRESSURE_MAX_VOLTAGE    = 2.71    # 10 mbar  — pump-down gate
PUMP_DOWN_COMPLETE_VOLTAGE   = 0.15    # ~0.01 mbar — high vacuum confirmed
ARGON_FLUSH_TARGET_VOLTAGE   = 1.287   # 0.09 mbar — plasma ignition pressure
SPUTTER_READY_TARGET_VOLTAGE = 0.363   # 0.007 mbar — default sputtering pressure
VENTING_COMPLETE_VOLTAGE     = 2.5     # VENTING -> IDLE threshold

# Pressure PID control
ARGON_FLUSH_FLOW_SETPOINT    = 150.0   # sccm — initial flow on flush entry
PRESSURE_CONTROL_KP          = 8.0     # sccm/V — proportional gain (tuned)
PRESSURE_CONTROL_KD          = 3.0     # sccm·s/V — derivative gain (tuned)
PRESSURE_CONTROL_KI          = 0.05    # sccm/(V·s) — integral gain (tuned down from 0.5, was unstable)
PRESSURE_CONTROL_ICLAMP      = 50.0    # sccm — anti-windup clamp on integral contribution

# Timing
PLASMA_IGNITION_TIMEOUT      = 120.0   # seconds — auto-abort if plasma not confirmed
POLLING_INTERVAL             = 0.2     # seconds between sensor reads (lowered from 0.5)
GUI_REFRESH_INTERVAL         = 200     # ms between GUI updates (lowered from 500)

# Hardware
GPIO_PIRANI_PIN              = 17      # BCM — turbo enable opto
GPIO_MFC_VALVE_CLOSE_PIN     = 27      # BCM — emergency valve close
GPIO_TURBO_VALVE_PIN         = 4       # BCM — turbo inlet valve opto (HIGH=closed, LOW=open)
ADS1115_I2C_ADDRESS          = 0x48
ARGON_DAC_I2C_ADDRESS        = 0x60
ARGON_DAC_VREF                = 3.3    # MCP4725's real ceiling — Pi 3.3V rail (buffer doesn't raise this)
MFC_FULL_SCALE               = 700.0   # sccm
MFC_GAS_CORRECTION_FACTOR    = 1.39    # Argon correction for MKS 1179A
```

### Tuning the Pressure PID Loop

Current tuned values (Kp=8, Kd=3, Ki=0.05) were arrived at empirically on hardware:

- Started at Kp=40 (original, P-only) — produced slow, large-amplitude oscillation
- Dropped to Kp=10 — oscillation reduced significantly but response was sluggish
- Raised to Kp=20, added Kd=3–7 — eliminated overshoot at the 0.09 mbar (ARGON_FLUSH) setpoint, but undershoot persisted at 0.007 mbar (SPUTTER_READY) since the same gains behave differently at very different flow regimes
- Settled at Kp=17→8 (further reduced after switching `POLLING_INTERVAL` from 0.5s to 0.2s, which made the derivative term ~2.5× more aggressive for the same physical Kd — **gains must be retuned whenever `POLLING_INTERVAL` changes**), Kd=7→3, with undershoot at 0.007 mbar reduced from 0.005 to 0.006 mbar (closer to target)
- I-term added to address a small persistent offset (target 0.06 mbar settling at ~0.055); Ki=0.5 caused significant overshoot, reduced to Ki=0.05 with anti-windup clamping (`PRESSURE_CONTROL_ICLAMP`) to prevent integral runaway when the MFC is saturated at the DAC ceiling

If you change `POLLING_INTERVAL`, expect to retune Kd and Ki — both depend on `dt` between samples.

---

## GUI Guide

| Element | Description |
|---|---|
| **Pi IP** | Live IP address, updates automatically on network change |
| **Process State** | Current state, colour-coded |
| **Start Argon Flush** | Opens MFC valve, starts PID loop to 0.09 mbar |
| **Confirm Plasma** | Manually confirms plasma ignition; transitions to SPUTTER_READY |
| **Start Sputter** | Begins sputtering from SPUTTER_READY |
| **Stop Sputter** | Ends sputtering, transitions directly to VENTING |
| **Vent** | Vents chamber to atmosphere |
| **E-STOP** | Immediate: kills flow, closes valve, drops opto, returns to IDLE |
| **Sputter P (mbar)** | Operator-set target pressure for SPUTTER_READY/SPUTTERING; press Set or Enter |
| **Argon PSI** | Enter upstream argon regulator pressure; required ≥ 15 psi to start flush/sputter; press Update or Enter |
| **Pirani voltage + Pressure (mbar)** | Live readings, mbar computed via log-linear calibration table |
| **Pirani graph** | Scrolling ADC voltage (lime, 0–4V, 60 samples) |
| **MFC graph** | Scrolling flow sccm (cyan) overlaid with DAC voltage (yellow dashed) |
| **OPTO** | Green = turbo interlock enabled; Red = off |
| **DAC OUT** | Green = DAC found on I2C; Red = not detected |
| **Valve** | Green = released (MFC in control); Red = closed (emergency) |

Start Sputter / Stop Sputter buttons share one grid cell and swap automatically based on state, rather than appearing side by side with one disabled.

---

## Known Limitations

- **DAC ceiling at 3.3V**: The MCP4725 is powered from the Pi 3.3V rail. Now buffered through an LM358P unity-gain follower (driven from 5V) so it can reliably *deliver* its full 0–3.3V range under load — but the DAC itself still cannot exceed 3.3V, capping commanded MFC flow at roughly 46% of full scale (~322 sccm) until a true level-shifted 5V DAC path is added.
- **Turbo opto failures (GPIO 17)**: Two 4N35s have failed in testing, suspected inductive kickback from the turbo controller input. No flyback diode is currently installed — recommended before further runs (see Hardware Overview).
- **Turbo inlet valve interlock is pressure-based, not RPM-based**: opens above 0.1 mbar regardless of actual turbo spindown state. A proper interlock needs the EXC120 analogue speed output wired to an ADC channel.
- **`_ignite_plasma()` is a stub**: RF power supply trigger is not yet implemented.
- **SPUTTERING state pressure control uses the same operator-set target as SPUTTER_READY**: there's no separate process recipe / ramp profile yet.
- **Pirani gauge readings may drift**: observed mismatch between commanded and indicated pressure even after confirming the control loop and calibration interpolation were correct — suspected gauge aging or thermal drift, not yet root-caused.

---

## Pending Features

- [ ] Flyback diode + pulldown on turbo enable opto circuit (GPIO 17) to prevent further 4N35 failures
- [ ] RPM-based turbo inlet valve interlock using EXC120 analogue speed output (pins 16/17), replacing the current pressure-based gate
- [ ] RF power supply trigger in `_ignite_plasma()`
- [ ] Current sensor integration for automatic plasma detection
- [ ] Plasma frame GUI controls (right panel — currently placeholder)
- [ ] DAC level shifter / dedicated 5V supply for full MFC range
- [ ] Datalog / CSV export of Pirani and MFC readings
- [ ] Per-state PID gain sets (ARGON_FLUSH and SPUTTER_READY have different dynamics)
- [ ] Investigate Pirani gauge drift/calibration mismatch
- [ ] Config hot-reload (was scoped, descoped this session — full code hot-reload would still require a restart for logic/GUI changes)
