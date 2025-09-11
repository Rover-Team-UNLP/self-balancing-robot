# Self-Balancing Robot (Inverted Pendulum)

## Project Overview

This project aims to build a **two-wheeled self-balancing robot**, similar to a Segway.  
The robot uses real-time sensor data to stay upright while being able to move forward and backward.

**Reference / Inspiration:**  
[A2 - Stabilize Sensor Readings With Kalman Filter](https://www.instructables.com/Stabilize-Sensor-Readings-With-Kalman-Filter/)

---

## Hardware

- **Microcontroller:** ESP32  
- **Sensor:** MPU6050 (gyroscope + accelerometer)  
- **Motors:** 2 DC geared motors  
- **Motor Driver:** TB6612FNG (or equivalent)  
- **Power Supply:** Li-ion battery pack / regulated voltage  

---

## Software & Real-Time Control

The ESP32 is responsible for:

1. **Reading sensor data** (accelerometer + gyroscope)  
2. **Calculating tilt angle** using a Kalman filter  
3. **Adjusting motor speed** hundreds of times per second

### Task Priorities

- **Critical Task (Balance Loop):**  
  Runs at precise intervals (e.g., every 4 ms) to read sensors, calculate tilt, and adjust motors.  

- **Low-Priority Task (Communication & Telemetry):**  
  - Sending telemetry data to a server or visualization tool  
  - Remote control with two buttons (forward/backward)

---

## License

This project is released under the MIT License.
