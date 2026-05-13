# PMSM-Motor-Drive
Embedded C implementation of PMSM motor control using TMS320F28379D DSP with eQEP-based rotor position feedback and SPWM inverter control.

// ************************************************************************************************************************************** //

# PMSM Motor Drive using TMS320F28379D

## Overview
This project implements a closed-loop PMSM motor drive using the Texas Instruments TMS320F28379D Digital Signal Processor (DSP). The system initially operates the motor in six-step BLDC mode using Hall sensor feedback and transitions to SPWM-based PMSM operation after reaching a predefined speed threshold.

The project focuses on low-cost PMSM control by utilizing Hall sensors for rotor position estimation instead of expensive encoders.

## Features
- Closed-loop PMSM motor drive
- Six-step BLDC startup control
- Automatic transition to PMSM/SPWM operation
- Hall-sensor-based rotor position estimation
- eQEP signal generation for rotor position emulation
- Real-time PWM generation using TMS320F28379D
- Inverter gate pulse generation using Smart Power Module
- Speed estimation and control

---

## Hardware Used
- Texas Instruments TMS320F28379D DSP
- BLDC/PMSM Motor
- Hall Sensors
- Smart Power Module (SPM)
- Three-phase inverter
- Gate driver circuit
- DC Power Supply

---

## Software Used
- Code Composer Studio (CCS)
- Embedded C

---

## Working Principle

### 1. BLDC Startup
The motor initially runs in six-step commutation mode using Hall sensor feedback to achieve reliable startup torque.

### 2. Mode Transition
Once the motor speed reaches approximately 10 Hz, the controller automatically transitions from BLDC operation to SPWM-based PMSM control.

### 3. Rotor Position Emulation
Hall sensor signals are processed to generate equivalent eQEP signals for rotor position estimation and smoother PMSM operation.

### 4. PWM Generation
The TMS320F28379D generates high-frequency PWM signals for inverter switching and motor control.

### 5. PMSM Operation
After position estimation, sinusoidal PWM / Space Vector Modulation is applied for smoother torque and improved efficiency.

---

## TMS320F28379D Modules Used
- GPIO Module
- ePWM Module
- eQEP Module
- ADC Module
- Timer Module
- Interrupt Module

---

## Folder Structure

```text
PMSM-Motor-Drive-TMS320F28379D/
│
├── src/            # Source files
├── waveforms/      # PWM and signal waveforms
└── README.md
