# Sputter Vacuum Controller

A Raspberry Pi-based vacuum process controller for DC magnetron sputter deposition. Manages chamber pump-down, argon gas flow, plasma ignition, and sputtering via a live Tkinter GUI with scrolling sensor graphs and a hardware-interlocked state machine.

---

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [Hardware Notes (Hardware Notes)](#hardware-notes)
- [Software Architecture](#software-architecture)
- [File Structure](#file-structure)
- [State Machine](#state-machine)
- [Pressure Reference](#pressure-reference)
- [Installation](#installation)
- [Running](#running)
- [Configuration](#configuration)
- [GUI Guide](#gui-guide)
- [Known Limitations](#known-limitations)
- [Roadmap](#roadmap)

---

## Hardware Overview

| Component | Model | Interface | Notes |
|---|---|---|---|
| Microcontroller | Raspberry Pi 2B (BCM GPIO) | — | Host for all control logic |
| Pirani gauge | ACE Instruments DHPG-015 | Analog 0–10V | Divided to 0–3.3V before ADC (ratio 0.33) |
| ADC | ADS1115 | **I2C bus 3** @ 0x48 | Pirani on A0, MFC feedback on A1, turbo RPM tach on A2 |
| MFC | MKS 1179A | Analog 0–5V setpoint | 0–700 sccm Argon |
| DAC | MCP4725 | **I2C bus 3** @ 0x60 | Powered off Pi 3.3V rail — buffered, see below |
| DAC buffer | LM358P op-amp | Analog | Unity-gain follower; required, DAC alone cannot drive MFC setpoint input |
| Turbo enable | Opto-isolator (4N35) | GPIO 17 (BCM, phys 11) | Marginal — needs BC547 output buffer, see below |
| MFC valve close | Emergency shut | GPIO 27 (BCM, phys 13) | Pulls MFC valve closed on demand |
| Turbo inlet valve | Relay via BC547 | **GPIO 22 (BCM, phys 15)** | HIGH = valve OPEN, LOW = valve CLOSED (relay NC contact) |

### I2C: Software Bus 3 on GPIO 23/24

The Pi's hardware I2C pads (GPIO 2/3, physical pins 3/5) were affected in the early hardware issue (see below). I2C now runs as a **bit-banged software bus** via device-tree overlay in `/boot/firmware/config.txt`:

```
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=23,i2c_gpio_scl=24,i2c_gpio_delay_us=2
```

| Signal | BCM | Physical pin |
|---|---|---|
| SDA | GPIO 23 | 16 |
| SCL | GPIO 24 | 18 |

The code opens the bus with `ExtendedI2C(I2C_BUS_NUMBER)` from `adafruit-extended-bus` (bus number set in `config.py`; switch back to `1` if hardware I2C is ever restored). **Note:** unlike pins 3/5, GPIO 23/24 have no on-board pull-ups — the breakout boards' 10 kΩ pull-ups are what keeps the bus alive. Bare chips would need external 4.7 kΩ pull-ups to 3.3 V.

Software bus speed is slower than hardware I2C; irrelevant at the 0.2 s polling rate. A harmless "I2C frequency is not settable in python" warning is printed at startup.

### Voltage Divider (Pirani Output)

The DHPG-015 outputs 0–10V. A 3-resistor voltage divider scales this to 0–3.3V for the ADS1115 (divider ratio 0.33). All pressure thresholds in `config.py` are expressed in post-divider (ADC-side) volts. The full manufacturer calibration table lives in `Docs/Di-Hi-Pr-Pirani output voltage.pdf` and is transcribed as `_PIRANI_CAL` in `main.py`.

### DAC Output Buffer (LM358P)

The MCP4725 cannot source enough current to drive the MFC setpoint input directly — above ~1.25V the output voltage collapses under load even though it reads correctly when unloaded. An LM358P wired as a unity-gain voltage follower buffers the DAC output before it reaches the MFC:

```
DAC OUT ──────────────┬──── LM358 IN+ (pin 3)
                       │
                  (feedback) IN- (pin 2) ──── OUT (pin 1) ──── MFC setpoint
LM358 VCC (pin 8) ── Pi 5V
LM358 GND (pin 4) ── GND
```

Power the op-amp from the Pi's **5V** rail, not 3.3V — at 3.3V supply the LM358's output headroom (~1.5V below V+) caps it at ~1.8V. At 5V supply the usable output range covers the full 0–3.3V DAC range. The buffer increases available *current*, not voltage.

### Turbo Inlet Valve (GPIO 22 → BC547 → Relay → Solenoid)

The turbo inlet solenoid valve (~50 mA @ 25–26 V) is switched by a relay, driven by a BC547:

```
GPIO22 ──470Ω── base   BC547   collector ── relay coil ── +5V
                          emitter ── GND         ▲
                                          1N4007 across coil
                                          (band toward +5V)

Valve circuit ── relay COM + NC contacts (isolated from the Pi)
```

- Valve is wired through the relay's **NC (normally closed)** contact: relay released = valve circuit shorted = **valve CLOSED**; relay energized = **valve OPEN**.
- Logic (in `_poll()`, `turbo_valve_step()`): valve held CLOSED at all times; **latched OPEN** during **VENTING** once turbo pump RPM (tach on ADC A2, see [Turbo Pump RPM](#turbo-pump-rpm-adc-a2)) sustains `TURBO_VALVE_CONFIRM_SAMPLES` consecutive reads at or below `TURBO_VALVE_OPEN_RPM_MAX` (20000 RPM); re-closed automatically on leaving VENTING. The debounce prevents a single corrupted I2C read from opening the valve; the latch prevents relay chatter once open. Driven by rotor speed directly rather than inferring it from chamber pressure — the earlier pressure-based gate (`TURBO_VALVE_OPEN_MBAR`) has been replaced.
- Fail-safe: if the Pi crashes or loses power, the relay releases and the valve **fails closed** — the safe direction for the turbo inlet.
- GUI shows live actuation state (`T-VALVE : CLOSED` / `OPEN (venting)`).

### Turbo Pump RPM (ADC A2)

- Tach/speed output, 0–3.3V linear = 0–90000 RPM (`TURBO_RPM_VOLTAGE_FULL_SCALE`, `TURBO_RPM_FULL_SCALE`), read by `TurboRPMController` (`turbo_rpm.py`) every poll tick regardless of state.
- Displayed in the Pirani panel, right edge (`T-RPM`, `T-RPM V`) — value and raw ADC voltage both shown.
- Feeds the turbo inlet valve interlock above; not used for any other control decision.

Do **not** drive relay coils or solenoids from a 4N35 directly — see the hard-won rule below.

### Opto/Load Rule of Thumb (learned the expensive way)

A 4N35's usable output current is roughly `CTR (~100%) × LED current (~10 mA max from a GPIO)` ≈ **10 mA**. Asked for more, it goes linear and drops the supply across itself (looks like a mystery series resistor). Loads measured on this rig:

| Load | Draw | 4N35 alone? |
|---|---|---|
| Turbo enable input | ~15–30 mA | **No** — needs BC547 buffer (pending) |
| Relay coil (5V) | ~60 mA | No — driven by BC547 directly, opto not needed (relay contacts provide the isolation) |
| Inlet solenoid | ~50 mA @ 26V | No — relay contacts switch it |

The turbo enable opto (GPIO 17) currently only works with excess LED drive; the proper fix is a BC547 buffer on its output (opto pin 5 → enable+, pin 4 → BC547 base, 10 kΩ base–emitter, BC547 C/E across the enable terminals). Flyback diodes go **in parallel across coils** (reverse-biased), never in series.

---

## Hardware Notes (Hardware Notes)

During valve wiring, the 26 V solenoid supply contacted the logic wiring around header pins 3–7. Notes — **do not use these pins**:

| Pin | BCM | Was | Status |
|---|---|---|---|
| phys 3 | GPIO 2 (SDA) | hardware I2C | **not used — bypassed by software bus 3 |
| phys 5 | GPIO 3 (SCL) | hardware I2C | **not used — bypassed by software bus 3 |
| phys 7 | GPIO 4 | turbo inlet valve | **not used — moved to GPIO 22 |

The original ADS1115 were affected during same event (replaced); the MCP4725 survived. Notes: 26 V wiring physically segregated from the logic breadboard (only relay contacts bridge the domains); series resistors into ADC inputs recommended (pending).

---

## Software Architecture

```
main.py
├── Hardware init (GPIO, ExtendedI2C bus 3, ADS1115)
├── Shared state dict (thread-safe, _lock)
├── Polling thread (_poll) — runs every POLLING_INTERVAL (0.2s)
│   ├── Reads Pirani + MFC + turbo RPM (A2) sensors
│   ├── Drives MFC pressure PID loop
│   ├── Drives turbo inlet valve latch (GPIO 22), gated on turbo RPM (not pressure)
│   ├── Calls sm.update() for auto-transitions
│   ├── Converts Pirani voltage <-> mbar via log-linear calibration table
│   ├── End-of-tick safety cutoff re-assert (E-STOP race guard)
│   └── Updates shared state dict
├── Tkinter GUI (main thread)
│   ├── _refresh() — runs every GUI_REFRESH_INTERVAL (200ms) via root.after()
│   ├── Scrolling graphs (ScrollingGraph)
│   ├── Live pressure readout in mbar (adc_voltage_to_mbar)
│   ├── Manual set-point entry fields (sputter target mbar, argon PSI)
│   ├── Error display with ERROR_DISPLAY_SECONDS auto-expiry
│   └── Button callbacks → sm.transition()
└── Hardware interlock callback
    └── handle_hardware_interlocks() — fires on every state transition
```

The polling thread and GUI thread never share objects directly — all data passes through `_state` dict protected by `_lock`. On window close, `_stop_event` is set and the polling thread is **joined** (2 s timeout) before hardware shutdown and `GPIO.cleanup()`.

**E-STOP race guard:** the poll loop reads the state once per tick; if an E-STOP/vent transition lands mid-tick, the interlock callback's cutoff could be overwritten by in-flight `valve_release()`/`set_flow()` calls. The poll loop therefore re-asserts the full safety cutoff at the end of any tick in which the state machine moved to IDLE/VENTING — the cutoff is always the last hardware write of the tick.

**Error handling:** errors are no longer cleared by the polling loop each tick. They persist in the status bar for `ERROR_DISPLAY_SECONDS` (10 s) and then auto-expire, so both operator-guard messages and the plasma-timeout message are actually readable.

---

## File Structure

```
Sputter_ctrl/
├── main.py            — Entry point: GUI, polling thread, hardware init, cal table
├── config.py          — All constants, thresholds, and pin assignments
├── state_machine.py   — SputterStateMachine class
├── mfc_control.py     — MFCController class (DAC, valve, PID loop)
├── pirani.py          — PiraniController class (ADC, turbo enable opto)
├── turbo_rpm.py       — TurboRPMController class (ADC A2 tach → RPM)
├── graph.py           — ScrollingGraph class (pure Tkinter, no matplotlib)
├── setup.sh           — venv + dependency installer
├── Docs/              — Pirani calibration datasheet PDF
└── tests/
    ├── adc_test.py         — ADS1115 read loop (bus 3)
    ├── dac_test.py         — MCP4725 sweep + readback (bus 3)
    ├── opto_test.py        — GPIO 17 turbo enable opto toggle
    └── turbo_valve_test.py — GPIO 22 relay/valve toggle
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

### Automatic Transitions

| From | To | Condition | Where |
|---|---|---|---|
| `IDLE` | `PUMP_DOWN` | Pirani voltage ≤ 2.71V (10 mbar) | `_poll()` |
| `PUMP_DOWN` | `IDLE` | Pirani voltage ≥ 2.71V (pump failure/leak) | `_poll()` + `sm.update()` |
| `PUMP_DOWN` | `READY` | Voltage ≤ 0.2V AND opto enabled | `sm.update()` |
| `READY` | `PUMP_DOWN` | Pirani voltage ≥ 2.71V (pressure degraded) | `sm.update()` |
| `ARGON_FLUSH` | `PLASMA_IGNITING` | Voltage ≥ 1.287V (0.09 mbar) | **`_poll()` only** — sole owner: arms the ignition timeout and fires `_ignite_plasma()` |
| `PLASMA_IGNITING` | `READY` | 120s timeout, no plasma confirmation | `_poll()` |
| `VENTING` | `IDLE` | Pirani voltage ≥ 2.5V | `sm.update()` |

`PLASMA_IGNITING → SPUTTER_READY` is **operator-only** (Confirm Plasma button) — there is deliberately no sensor-driven path, since there is no plasma detection hardware yet.

### Manual Transitions (Operator Buttons)

| Button | Transition | Guard |
|---|---|---|
| Start Argon Flush | `READY → ARGON_FLUSH` | Argon inlet ≥ 15 psi |
| Confirm Plasma | `PLASMA_IGNITING → SPUTTER_READY` | None |
| Start Sputter | `SPUTTER_READY → SPUTTERING` | Argon inlet ≥ 15 psi |
| Stop Sputter | `SPUTTERING → VENTING` | None |
| Vent | Most states → `VENTING` | None |
| E-STOP | Any → `IDLE` | None — immediate |

### Hardware Interlock Callback

`handle_hardware_interlocks()` fires synchronously on every state transition:

- **→ IDLE or VENTING**: `mfc.set_flow(0)`, `mfc.valve_close()`, `pirani.set_opto(False)`, and resets `plasma_ignition_start` / `plasma_ignition_triggered` so the next flush cycle starts clean (a stale flag previously could leave PLASMA_IGNITING with no armed timeout)
- **→ PUMP_DOWN**: `pirani.set_opto(False)` — resets opto for next pump-down cycle

During VENTING the poll loop additionally holds the MFC valve **closed** every tick (venting is done with the turbo inlet valve, not through the MFC).

---

## Pressure Reference

All voltages are post-divider (ADC input, ×0.33 from raw gauge output). Conversion between Pirani voltage and mbar uses **log-linear interpolation** against the full manufacturer calibration table (`_PIRANI_CAL` in `main.py`, source PDF in `Docs/`): linear in voltage, logarithmic in pressure, matching the gauge's thermal-conductivity response. Both directions are implemented (`mbar_to_adc_voltage`, `adc_voltage_to_mbar`) and round-trip to machine precision at all 51 table points.

| Pressure (mbar) | Raw gauge (V) | ADC input (V) | Significance |
|---|---|---|---|
| 999 | 10.00 | 3.30 | Atmosphere (ADC saturated) |
| 10 | 8.20 | 2.706 | `IDLE_PRESSURE_MAX_VOLTAGE` — pump-down gate |
| 0.09 | 3.90 | 1.287 | `ARGON_FLUSH_TARGET_VOLTAGE` — plasma ignition pressure |
| ~0.09 | 3.94 | 1.3 | Turbo enable opto fires (PUMP_DOWN, transition-based) |
| ~0.0037 | 0.61 | 0.2 | `PUMP_DOWN_COMPLETE_VOLTAGE` — PUMP_DOWN → READY gate |
| 0.007 | 1.10 | 0.363 | 0.007 mbar — default sputtering pressure |
| 2.5 (ADC volts) | — | 2.5 | `VENTING_COMPLETE_VOLTAGE` — VENTING → IDLE |

The GUI displays live pressure in mbar (bold, under the voltage readout), computed via `adc_voltage_to_mbar()`; reads `ATM (>999 mbar)` at ADC saturation.

### Sputter Target Pressure (operator-settable)

An input field sets the SPUTTER_READY / SPUTTERING target pressure directly in mbar (default 0.007). Entered values are converted to an ADC voltage target via the calibration table before being passed to `pressure_control_step()`.

### Turbo Enable Opto Logic (GPIO 17)

Transition-based, not level-based:

- Goes **HIGH** the first time Pirani voltage drops below **1.3V** during `PUMP_DOWN`
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

### 2. Enable the Software I2C Bus

Hardware I2C (pins 3/5) is dead on this Pi — the software bus overlay is **required**. Append to `/boot/firmware/config.txt` and reboot:

```bash
echo "dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=23,i2c_gpio_scl=24,i2c_gpio_delay_us=2" | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

Verify both devices are visible:

```bash
i2cdetect -y 3
# Should show 0x48 (ADS1115) and 0x60 (MCP4725)
```

### 3. Run the Setup Script

```bash
cd ~/Sputter_ctrl
chmod +x setup.sh
./setup.sh
```

This installs `adafruit-blinka`, `adafruit-circuitpython-ads1x15`, `adafruit-extended-bus`, and `RPi.GPIO` into a venv at `~/Sputter_ctrl/venv/`, and adds a `source_sputt` alias to `~/.bashrc`.

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

Bench-test scripts (run these with `main.py` stopped — they fight over pins):

```bash
python tests/adc_test.py          # live ADC readings, both channels
python tests/dac_test.py          # DAC voltage staircase + register readback
python tests/opto_test.py         # GPIO 17 toggle every 5 s
python tests/turbo_valve_test.py  # GPIO 22 relay/valve toggle every 5 s
```

---

## Configuration

All tunable parameters are in `config.py`. Key constants:

```python
# I2C
I2C_BUS_NUMBER               = 3       # software i2c-gpio bus (1 = dead hardware bus)
ADS1115_I2C_ADDRESS          = 0x48
ARGON_DAC_I2C_ADDRESS        = 0x60

# GPIO (BCM)
GPIO_PIRANI_PIN              = 17      # turbo enable opto
GPIO_MFC_VALVE_CLOSE_PIN     = 27      # emergency valve close
GPIO_TURBO_VALVE_PIN         = 22      # turbo inlet valve relay (was GPIO 4 — dead pad)

# Pressure thresholds (post-divider volts unless noted)
IDLE_PRESSURE_MAX_VOLTAGE    = 2.71    # 10 mbar  — pump-down gate
PUMP_DOWN_COMPLETE_VOLTAGE   = 0.2     # PUMP_DOWN → READY gate
ARGON_FLUSH_TARGET_VOLTAGE   = 1.287   # 0.09 mbar — plasma ignition pressure
VENTING_COMPLETE_VOLTAGE     = 2.5     # VENTING → IDLE threshold

# Turbo pump RPM (tach on ADC A2) and inlet valve interlock
ADC_CHANNEL_TURBO_RPM        = 2
TURBO_RPM_VOLTAGE_FULL_SCALE = 3.3     # volts at A2 for full-scale RPM
TURBO_RPM_FULL_SCALE         = 90000.0 # RPM at full-scale voltage
TURBO_VALVE_OPEN_RPM_MAX     = 20000   # inlet valve opens once RPM drops to/below this
TURBO_VALVE_CONFIRM_SAMPLES  = 3       # consecutive reads required before opening (debounce)

# Pressure PID control
ARGON_FLUSH_FLOW_SETPOINT    = 150.0   # sccm — initial flow on flush entry
PRESSURE_CONTROL_KP          = 8.0     # sccm/V — proportional gain (tuned)
PRESSURE_CONTROL_KD          = 3.0     # sccm·s/V — derivative gain (tuned)
PRESSURE_CONTROL_KI          = 0.005   # sccm/(V·s) — integral gain
PRESSURE_CONTROL_ICLAMP      = 50.0    # sccm — anti-windup clamp (KI=0 is safe)

# Timing
PLASMA_IGNITION_TIMEOUT      = 120.0   # seconds — auto-abort if plasma not confirmed
POLLING_INTERVAL             = 0.2     # seconds between sensor reads
GUI_REFRESH_INTERVAL         = 200     # ms between GUI updates
ERROR_DISPLAY_SECONDS        = 10      # status-bar error auto-expiry

# MFC / DAC
MFC_FULL_SCALE               = 700.0   # sccm
MFC_GAS_CORRECTION_FACTOR    = 1.39    # Argon correction for MKS 1179A
ARGON_DAC_VREF               = 5.0
ARGON_DAC_RESOLUTION         = 4096
```

### Tuning the Pressure PID Loop

Current tuned values (Kp=8, Kd=3) were arrived at empirically on hardware:

- Started at Kp=40 (original, P-only) — produced slow, large-amplitude oscillation
- Dropped to Kp=10 — oscillation reduced significantly but response was sluggish
- Raised to Kp=20, added Kd=3–7 — eliminated overshoot at the 0.09 mbar (ARGON_FLUSH) setpoint, but undershoot persisted at 0.007 mbar (SPUTTER_READY) since the same gains behave differently at very different flow regimes
- Settled at Kp=17→8 (further reduced after switching `POLLING_INTERVAL` from 0.5s to 0.2s, which made the derivative term ~2.5× more aggressive for the same physical Kd — **gains must be retuned whenever `POLLING_INTERVAL` changes**), Kd=7→3
- I-term added to address a small persistent offset; large Ki caused overshoot, kept small with anti-windup clamping (`PRESSURE_CONTROL_ICLAMP`). Setting Ki=0 while tuning is safe (no divide-by-zero).

If you change `POLLING_INTERVAL`, expect to retune Kd and Ki — both depend on `dt` between samples.

---

## GUI Guide

| Element | Description |
|---|---|
| **Pi IP** | Live IP address shown in header |
| **Process State** | Current state, colour-coded |
| **Start Argon Flush** | Opens MFC valve, starts PID loop to 0.09 mbar |
| **Confirm Plasma** | Manually confirms plasma ignition; transitions to SPUTTER_READY |
| **Start Sputter** | Begins sputtering from SPUTTER_READY |
| **Stop Sputter** | Ends sputtering, transitions directly to VENTING |
| **Vent** | Vents chamber to atmosphere |
| **E-STOP** | Immediate: kills flow, closes valve, drops opto, returns to IDLE |
| **Voltage + Pressure (mbar)** | Live Pirani readings; mbar via log-linear calibration table |
| **OPTO** | Green = turbo enable opto on; Red = off |
| **T-VALVE** | Green `CLOSED` = inlet valve shut (normal); Orange `OPEN (venting)` = venting and turbo RPM has dropped to/below `TURBO_VALVE_OPEN_RPM_MAX` |
| **T-RPM / T-RPM V** | Right edge of the Pirani panel; turbo pump RPM (ADC A2) and its raw ADC voltage, read every tick |
| **Sputter P (mbar)** | Operator-set target pressure; press Set or Enter |
| **Argon PSI** | Operator-entered regulator pressure; ≥ 15 psi required for flush/sputter |
| **Pirani graph** | Scrolling ADC voltage (lime, 0–4V, 60 samples) |
| **MFC graph** | Scrolling flow sccm (cyan) overlaid with DAC voltage (yellow dashed) |
| **DAC OUT** | Green = DAC found on I2C; Red = not detected |
| **Valve** | Green = released (MFC in control); Red = closed (emergency) |
| **Status bar** | Errors shown in red, auto-expire after 10 s |

---

## Known Limitations

- **Three dead GPIO pads** (GPIO 2, 3, 4) from the early hardware issue — see [Hardware Notes](#hardware-notes). Software I2C bus 3 and GPIO 22 are the workarounds; a replacement Pi would allow reverting to hardware I2C.
- **Turbo enable opto (GPIO 17) is marginal**: the enable input draws ~15–30 mA, beyond a 4N35's ~10 mA saturated capability at legal GPIO LED drive. Currently only works with excess LED current. Proper fix (BC547 output buffer) designed but not yet installed.
- **DAC ceiling at 3.3V**: MCP4725 powered from the Pi 3.3V rail, buffered through the LM358P for current but capped at 3.3V — commanded MFC flow limited to roughly 46% of full scale (~322 sccm) until a level-shifted 5V DAC path is added.
- **MFC feedback (A1) scaling unverified**: the MKS 1179A feedback is 0–5V; whether A1 has a divider (and therefore what full-scale voltage means) has not been confirmed — displayed flow may be scaled wrong, and an undivided 5V input would over-stress the ADS1115.
- **Argon PSI guard is honor-system**: the ≥15 psi check reads an operator-entered value, not a sensor, and the entered value persists across runs.
- **No series protection resistors on ADC inputs yet** — recommended after general input protection.
- **`_ignite_plasma()` is a stub**: RF power supply trigger is not yet implemented.
- **Pirani gauge readings may drift**: at atmosphere A0 has been observed at ~3.04V where the table expects ~3.3V — either partial vacuum at time of reading, or the divider ratio is slightly under 0.33. Not yet root-caused.

---

## Roadmap

Goals only, near-term → eventual. Inspired in part by how production fabs structure
tool control: hardwired interlocks under software, standardized state/alarm/recipe
models (SECS/GEM), and fault detection built on ruthless data collection.

### Harden the platform

- [ ] **Software armor**: single safety gate at the top of `_poll()`, startup hardware
      self-test, watchdog (heartbeat + Pi hardware watchdog), persistent error log,
      config validation at launch
- [ ] **Simulation mode**: fake-hardware flag so the full state machine + GUI run on
      any laptop (the unit suite in `tests/unit/` already fakes the hardware layer;
      extend it into a live interactive sim)
- [ ] **Trustworthy analog front-end**: protected, calibrated sensor inputs with the
      divider ratios measured rather than assumed (Pirani atmosphere anomaly resolved)

### Make it a deposition machine

- [ ] **First characterized film**: complete a full run and measure the result — the
      milestone everything above serves
- [ ] **Full plasma integration**: supply trigger in `_ignite_plasma()` (with arc
      suppression confirmed in hardware), automatic plasma detection via current
      sensing, plasma GUI panel replacing the placeholder
- [x] **True turbo protection**: RPM-based inlet valve interlock (ADC A2, tach
      output), replacing the pressure-based gate — implemented in software
      (`turbo_valve_step()`, `TURBO_VALVE_OPEN_RPM_MAX`); not yet validated
      against a real vent cycle on hardware
- [ ] **Full-range MFC control**: level-shifted 5V DAC path removing the ~46%
      flow ceiling
- [ ] **Per-state PID gain sets**: ARGON_FLUSH and SPUTTER_READY dynamics differ;
      gains should switch with state

### Operate like a fab tool

- [ ] **Data collection & FDC-lite**: log every run (pressure/flow/DAC traces),
      build known-good baselines, flag deviation — drift detection instead of
      surprise failures; dashboarding via Grafana/InfluxDB is the natural backend
- [ ] **Process recipes**: runs defined as recipe files (target pressure, power,
      time, ramp profiles) executed start-to-finish — repeatability over button-pressing
- [ ] **Alarm model**: separate operator-info messages from latched alarms that
      require acknowledgement, GEM-style
- [ ] **Maintenance counters**: target erosion time, pump hours, cycles since vent —
      scheduled maintenance instead of run-to-failure

### Eventual

- [ ] **Unattended operation**: headless mode, remote monitoring and alarm
      notifications — the point where the watchdog and plasma detection stop being
      optional
- [ ] **Host interface**: expose state/data/recipes over the network (simple JSON
      socket, or the `secsgem` Python library for a real SEMI E30 interface) so the
      tool can be driven like production fab equipment
- [ ] **Run-to-run control**: adjust the next recipe from measurements of the last
      film — closing the outermost loop
- [ ] **MCU delegation layer**: when commercial smart boxes (MFC, supply) get
      replaced by homebuilt hardware, delegate the fast loops to a dedicated
      microcontroller under the Pi (the SputterOS-shaped slot in this architecture)
- [ ] **Replicable kit**: documentation and BOM good enough that another lab can
      build this machine from the repo alone
