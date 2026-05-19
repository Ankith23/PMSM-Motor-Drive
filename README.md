# PMSM Motor Drive — TMS320F28379D

Hybrid BLDC-to-PMSM motor drive in bare **Embedded C** on the TI TMS320F28379D (C2000 Delfino, 200 MHz).
No HAL. No CubeMX. Pure register-level peripheral configuration.

Oscilloscope captures of PWM waveforms included in `WAVEFORMS.zip`.

---

## How it works

| Speed | Mode | Method |
|---|---|---|
| < 10 Hz | BLDC | Hall sensor 6-step commutation (GPIO52 / GPIO94 / GPIO111) |
| ≥ 10 Hz | PMSM | Sinusoidal SPWM using eQEP-derived rotor position angle |

**Frequency estimation** — CpuTimer0 measures the period between rising edges on H1 (GPIO52). A rolling average filters noise. When frequency crosses 10 Hz, the controller switches from `BLDCMOTOR()` to `NEWBLDCMOTOR()`.

**eQEP emulation** — No physical encoder used. ePWM2 and ePWM3 generate synthetic EQEP1A/EQEP1B quadrature signals with 90° phase offset via `TBPHS`. The eQEP module counts these; the latched `QPOSSLAT` register provides the rotor angle.

**SPWM generation** — Three sinusoidal duty cycles (0°, 120°, 240°) computed from eQEP position each cycle and written to ePWM4/5/6 CMPA and CMPB registers:
```
CMPA = 0.5 × TBPRD × (1 + sin(θ))
```

---

## Peripherals used

| Peripheral | Role |
|---|---|
| ePWM1 | Strobe pulse generation |
| ePWM2 / ePWM3 | Synthetic EQEP1A / EQEP1B quadrature signal generation |
| ePWM4 / ePWM5 / ePWM6 | 3-phase inverter gate drives (BLDC and PMSM modes) |
| eQEP1 | Position counter from emulated quadrature signals |
| ADC-A (SOC2/3/4) | Phase current sensing — ADCINA2, ADCINA3, ADCINA4 |
| CpuTimer0 | Hall edge period measurement for frequency estimation |
| GPIO52 / GPIO94 / GPIO111 | Hall sensor inputs H1 / H2 / H3 |

---

## Hardware

- TMS320F28379D LaunchPad (LAUNCHXL-F28379D)
- 3-phase BLDC / PMSM motor
- Smart Power Module (SPM) — 3-phase inverter + gate drivers
- Hall effect sensors × 3
- DC power supply

---

## Build

- **IDE:** Code Composer Studio v12+
- **Compiler:** TI C2000 CGT
- **Device files:** C2000Ware

Import into CCS, select TMS320F28379D target, build and flash via onboard XDS110 JTAG.

---

## Repository contents

| File | Description |
|---|---|
| `main.c` | Full firmware — peripheral init, control loop, ISRs |
| `WAVEFORMS.zip` | Oscilloscope captures of PWM and Hall signals |

---

## Author

**Ankith S Kashyap** — B.E. EEE, NIE Mysore (2025)
Targeting embedded firmware / automotive embedded roles.
[LinkedIn](https://linkedin.com/in/your-profile) · [Email](mailto:ankithskashyap23@gmail.com)
