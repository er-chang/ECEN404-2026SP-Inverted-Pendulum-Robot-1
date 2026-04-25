# Inverted Pendulum Robot
<p align="center">
  <img src="https://github.com/user-attachments/assets/39a3619b-3f00-4001-ab31-49569472c721" alt="group" height="250" />
  <img src="https://github.com/user-attachments/assets/3a0d0305-3216-443b-be6a-4b7adc43d2b6" alt="robot" height="250" />
</p>
<!-- <img width="3693" height="2638" alt="group" src="https://github.com/user-attachments/assets/39a3619b-3f00-4001-ab31-49569472c721" />
<img width="1105" height="1424" alt="robot" src="https://github.com/user-attachments/assets/3a0d0305-3216-443b-be6a-4b7adc43d2b6" /> -->

---

## Problem Statement
Autonomous warehouse robots generally experience instability when handling unbalanced loads or making sudden maneuvers. These movements can cause tilting, making balance a key challenge.

## Proposed Solution
* Deliver an autonomous robot that balances an unstable payload and is able to navigate through confined spaces.
* Integrate mechanical, sensing, power, and control subsystems into a cohesive, microcontroller-based platform designed for reliability and safety.

### Key Features
* **Active Balancing:** 1kHz high-speed PID control loop to maintain upright stability of a 1.5kg load.
* **Skid-Steer Navigation:** differential mixing and linear PWM deadzone bridging for zero-radius pivots.
* **Acoustic Ranging:** Digital Signal Processing (median filter) on HC-SR04 sensors for obstacle detection and distance holding.
* **Wireless Telemetry:** High-speed SPI data transfer between the microcontroller and an asynchronous ESP32 web server for live data visualization.

---

## Hardware Architecture
The system is structured to isolate real-time control processes from wireless telemetry and communication to maintain stable operation.

* **Main Controller:** STM32 Nucleo-F767ZI
  * Handles all sensor reading (ADC/I2C), PID calculations, and motor PWM generation.
  * Maintains consistent real-time execution (< 1.0ms loop time).
* **Telemetry Co-Processor:** ESP32
  * Receives telemetry via SPI communication protocol.
  * Hosts web server to display pitch angle, effort, and network diagnostics.
  * Features a command interface that remotely controls the system.
* **Sensors & Actuators:**
  * WH148 5K Potentiometer
  * 4x HC-SR04 Ultrasonic Sensors (Front, Left, Right, Back)
  * 4x DC Gear Motors

---

## Control Theory
The primary stabilization logic relies on a PID control algorithm with safeguards against integrator windup. We have also developed and simulated a theoretical model of the system's dynamics.

* **State Space Derivation:** A full mathematical breakdown of the system matrices and stability criteria can be found in the repository: [`InvertedPendulum_Controller_Math.pdf`](./InvertedPendulum_Controller_Math.pdf).
* **Turning Dynamics:** Lateral static friction is overcome using a calculated `PWM_DEADZONE` bridge (68 baseline) and a linear ramp function to prevent stalling during yaw commands.

---
## Setup
1. **Power Initialization:** Turn on the power switch to boot up the STM32 and ESP32. Ensure that the robot is supported in an upright, 0-degree position during boot up for rod angle calibration.
2. **Est. Telemetry Link:** On the host device, connect to the ESP32's local network:
   * **SSID:** `IPR_Robot` 
   * **Password:** `TAMUIPR1`
3. **Configure System:** Open the web dashboard **`http://192.168.4.1`** in your browser and select the desired mode of operation.
4. **Actuation:** Arm the motors via the web interface to put the system in active mode.
5. **Live Calibration:** Utilize the dashboard to tune control parameters and observe telemetry in real time.

> ** Built-in Safety Mechanisms**
> To prevent hardware damage and ensure safe operation, the system features several fail-safes:
> * **Hardware Override:** Motors can be manually disarmed at any time via the remote web dashboard.
> * **Software Kill Switch:** If the pendulum's pitch angle exceeds an unrecoverable **25 degrees**, the state machine forces motor effort to zero to prevent the wheels from aggressively accelerating across the floor.
