# 4-DOF MeArm Robotic Arm

This project showcases a 4-DOF robotic arm using the MeArm kit, programmed to perform pick-and-place tasks. The arm is driven by SG90 servo motors and controlled through an Arduino Uno with a PCA9685 PWM driver. Power is managed with a DC-DC buck converter, stepping down from 12V to 5V. MATLAB was used for kinematic analysis, trajectory visualization, and workspace planning.

---

## Features 

- Pre-assembled MeArm kit used for mechanical structure.
- Control system implemented using Arduino Uno and PCA9685.
- DC-DC converter ensures safe 5V power for servos from a 12V source.
- MATLAB used to simulate motion and validate kinematic models.
- Capable of smooth and repeatable pick-and-place operations.

---

## Hardware Components

| Component                | Description                             |
|--------------------------|-----------------------------------------|
| MeArm Kit (4-DOF)        | Pre-built acrylic structure              |
| 4× SG90 Servo Motors     | For joint actuation                     |
| Arduino Uno              | Microcontroller board                   |
| PCA9685 PWM Driver       | 16-channel PWM controller               |
| DC-DC Buck Converter     | Converts 12V input to 5V servo output   |
| 12V Battery/Adapter      | Power source                            |
| Breadboard & Wires       | For prototyping and connections         |

---

## Wiring Overview

- **Servo Motors** connected to PCA9685 outputs (channels 0–3).
- **PCA9685** connected to Arduino Uno via I2C (A4 for SDA, A5 for SCL).
- **PCA9685 V+** powered using the **DC-DC buck converter** output (5V).
- **DC-DC Input** powered by an external 12V battery or adapter.
- Common ground shared between Arduino, PCA9685, and power supply.

---

## MATLAB Analysis

- Defined DH parameters for forward and inverse kinematics.
- Visualized joint positions and simulated end-effector motion.
- Performed trajectory planning and workspace exploration.
- Used PD computed torque control simulation to evaluate smoothness.

> MATLAB scripts are included in the `/matlab` directory.

---

![MeArm Full View]( photo_2025-05-16_15-19-22.jpg )

![Circuit Connections](photo_2025-05-16_15-31-35.jpg)

---

