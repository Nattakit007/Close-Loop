<h1 align="center">Closed-Loop PID Controller</h1>

<h3 align="center">STM32 • Feedback Systems • PID Control • Hardware Dynamics</h3>

<p align="center">
  <img src="https://readme-typing-svg.demolab.com?font=Fira+Code&size=20&pause=1000&color=2F80ED&center=true&vCenter=true&width=650&lines=STM32+Closed-Loop+Control+System;Discrete+PID+Algorithm+Implementation;Sensor+Feedback+%26+System+Response;Bridging+Firmware+and+Physical+Dynamics" alt="Typing SVG" />
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Platform-STM32-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white" />
  <img src="https://img.shields.io/badge/Language-C%20%2F%20C%2B%2B-00599C?style=for-the-badge&logo=cplusplus" />
  <img src="https://img.shields.io/badge/Control-Closed--Loop-2F80ED?style=for-the-badge" />
</p>

---
NIGGA
### Overview

This project implements a real-time closed-loop control system on an STM32 microcontroller. The primary goal is exploring the practical bridge between digital algorithms and physical hardware behavior, analyzing how discrete controllers respond to continuous physical plants.

> *"The goal is not just to make the code work, but to understand what is actually happening between the controller, hardware, and physical system."*[cite: 1]

---

### Core Focus Areas

* **Feedback Systems**: Reading sensor feedback to close the execution loop in discrete time[cite: 1].
* **PID Control**: Calculating proportional, integral, and derivative terms to minimize tracking error[cite: 1].
* **Controller Tuning**: Adjusting gains to balance rise time, minimize overshoot, and eliminate steady-state error[cite: 1].
* **Sensor Feedback**: Acquiring and conditioning incoming measurement signals[cite: 1].
* **System Response**: Analyzing physical system dynamics across step inputs and continuous perturbations[cite: 1].
* **Real-World Hardware Behavior**: Mitigating non-ideal effects like actuator saturation, noise, and propagation delay[cite: 1].

---

### Control Topology

```mermaid
flowchart LR
    R[Target Setpoint r] --> Sum((+))
    Sum -->|Error e| PID[Discrete PID]
    PID -->|Control Signal u| Plant[Actuator / Plant]
    Plant --> Sensor[Sensor Feedback]
    Sensor -->|-| Sum

    style Sum stroke:#2F80ED,stroke-width:2px
    style PID stroke:#2F80ED,stroke-width:2px
    style Plant stroke:#2F80ED,stroke-width:2px
    style Sensor stroke:#2F80ED,stroke-width:2px
```