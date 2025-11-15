# DelTheta

## Overview

DelTheta is a smart automation solution for the real-time monitoring and control of Torpedo Ladle Car (TLC) tilt angles during hot metal pouring operations in steel plants. Developed for the Smart India Hackathon 2025, DelTheta transforms steel pouring into a safer, data-driven process, leveraging dual laser-based inclinometers, an intuitive HMI, and predictive alarms to prevent over-tilting and spillage.

## Problem Statement

Hot metal pouring in steel plants is hazardous due to operator error and crude tilt control, leading to dangerous spills and material wastage. DelTheta addresses this by providing:
- Real-time position feedback for TLC tilt
- Automated control to prevent over-tilting
- Integration of safety alerts and audit logs for compliance

## Features

- **Dual-redundant laser-based inclinometers:** High-accuracy, fail-safe tilt measurement ensures continuous monitoring[1].
- **Real-time HMI:** Visualizes tilt angles, actuator pressure, and torque so operators can make informed adjustments.
- **Colour-coded safety zones:** Green (safe), Amber (warning), Red (lock) warnings with visual and audio alarms.
- **Integrated smart monitoring:** Data logging, predictive alerts, and compliance auditing.
- **Operator support:** Intuitive alerts, ergonomic interface, and decision-support features.

## Technologies Used

- **Hardware:**
  - Laser-based inclination sensors
  - Microcontroller (for real-time data processing)
  - Industrial alarms and indicators
  - HMI (Human Machine Interface)
- **Software:**
  - Arduino IDE for microcontroller code
  - MATLAB for advanced processing and analysis
  - Custom HMI application

## System Architecture

```
[Torpedo Ladle] 
      │
[Dual Laser Inclinometers]  
      │
[Microcontroller]  
      │
[HMI + Alarms]  
      │
[Operator Feedback & Control]  
```

## How It Works

1. **Sensors** continuously transmit angular position from both sides of the ladle.
2. **Microcontroller** processes inputs and compares angles to predefined safe limits.
3. **HMI** displays real-time data; visual and audio alarms alert the operator if tilt angle approaches critical zones.
4. **Operator** uses interface for monitoring and can take immediate corrective action using actionable insights[1].

## Key Benefits

- **Enhanced safety:** Prevention of uncontrolled pouring reduces worker injuries.
- **Material savings:** Avoids loss and scrap from spills.
- **Steel quality improvement:** Maintains optimal flow and reduces slag carryover.
- **Regulatory compliance:** Complete audit trail of tilt actions and alerts.
- **Sustainability:** Decreased waste and emissions via spill prevention.

## Installation & Deployment

Requirements:
- Arduino-compatible microcontroller
- Laser-based inclinometer modules
- Industrial HMI display & alarm unit
- MATLAB (optional for advanced analytics)

Instructions:
1. Mount inclinometers on TLC body.
2. Connect to microcontroller and configure with provided firmware.
3. Deploy HMI application and set up alarms/indicators.
4. Train operators using included training modules.
5. Begin monitoring and use real-time data for tilting operations.

## Challenges & Solutions

- **Sensor redundancy**: Backup inclinometer prevents measurement failure.
- **Accuracy:** Calibrated compensation for sensor least count built into HMI.
- **Reliability:** Heat shielding and EMI protection for components.
- **Training:** Comprehensive modules provided for operator understanding.

## References

- [List of academic and technical references as cited in the project report][1].

## License

This project is developed by Team Error404z for Smart India Hackathon 2025.

***

For detailed documentation and references, refer to the full project PDF[1].

Citations:
[1] Error404z-TLC.pdf https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/88969593/592f08fc-4288-455e-959c-470653d5ed25/Error404z-TLC.pdf
