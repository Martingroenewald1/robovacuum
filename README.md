# RoboVacuum MPC

A custom Model Predictive Control (MPC) and perception stack for a differential drive robot vacuum, built entirely from scratch without ROS.

## Architecture

This project splits the high-level trajectory optimization from the low-level hardware actuation to ensure real-time safety and deterministic execution:

*   **High-Level Control (Python - 10Hz):** Handles LiDAR point cloud parsing, local obstacle avoidance, and the Model Predictive Control (MPC) optimization solver. Outputs target $v$ (linear) and $\omega$ (angular) velocities.
*   **Low-Level Control (Embedded C/C++ - 100Hz):** A strict real-time PID loop running on a microcontroller. It tracks the velocity commands using wheel encoders and enforces hardware watchdogs.

## Development Roadmap

- [ ] **Phase 1: Python SIL Simulation**
  - [x] Repository setup
  - [ ] 2D LiDAR raycasting emulator and environment generation
  - [ ] Differential drive kinematic modeling
- [ ] **Phase 2: Optimization Engine**
  - [ ] Implement Unscented Kalman Filter (UKF) for state estimation
  - [ ] Build the MPC solver with non-holonomic constraints
- [ ] **Phase 3: Hardware Integration**
  - [ ] C++ standalone 100Hz PID loop execution
  - [ ] IPC binary serial framing (COBS) between Python and the MCU

## Getting Started

*(Instructions for setting up the Python virtual environment and running the emulator will go here as the codebase develops.)*

---
**Author:** Martin Groenewald