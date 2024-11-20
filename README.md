

# Ball Balancer System

This project is an Arduino-based ball balancer system that dynamically adjusts the surface angle to stabilize a ball at a desired position. The system uses real-time feedback from an encoder to measure the ball's position and velocity, and it employs a PID control loop to make precise adjustments to the motor, ensuring that the ball remains balanced.

## Table of Contents

- [Introduction](#introduction)
- [Components](#components)
- [Circuit Diagram](#circuit-diagram)
- [How It Works](#how-it-works)
- [Code Overview](#code-overview)
- [Setup and Usage](#setup-and-usage)

## Introduction

The ball balancer system is designed to maintain the position of a ball on a platform by adjusting the platform's angle. It uses an encoder to measure the ball's movement and a motor to control the angle. The system's primary function is to keep the ball stable, even when external disturbances are introduced.

## Components

- **Arduino**: The microcontroller that runs the control algorithm.
- **Encoder**: Measures the ball's position and velocity on the platform.
- **Motor Driver**: Controls the motor speed and direction based on the control algorithm.
- **DC Motor**: Adjusts the platform angle to balance the ball.
- **Power Supply**: Powers the Arduino, motor, and encoder.

## Circuit Diagram

(Include a circuit diagram image here, showing how the components are connected.)

## How It Works

1. **Encoder Feedback**: The encoder provides real-time feedback on the ball's position and velocity by counting the encoder pulses as the ball moves.
2. **Control Algorithm**: The Arduino processes the encoder data and calculates the necessary adjustments using a PID (Proportional, Integral, Derivative) control algorithm.
3. **Motor Adjustment**: Based on the control signal calculated by the PID loop, the motor driver adjusts the motor's speed and direction to correct the platform's angle, keeping the ball balanced.
4. **Filtering**: A low-pass filter is applied to the velocity readings to reduce noise and improve the system's stability.

## Code Overview

The code is structured into several key functions:

- `setup()`: Initializes the pins, sets up the encoder, and attaches interrupts for precise position measurement.
- `loop()`: Continuously reads the encoder data, calculates the velocity, and applies the PID control algorithm to adjust the motor.
- `setMotor()`: Controls the motor speed and direction based on the computed control signal.
- `readEncoder()`: Interrupt service routine that updates the ball's position by reading the encoder.

### PID Control

The PID control loop is essential for stabilizing the ball. It compares the current velocity with the target velocity and adjusts the motor power accordingly:

```cpp
float kp = 5;
float ki = 0;
float e = target - v1Filt;
eintegral = eintegral + e * deltaT;

float u = kp * e + ki * eintegral;
```

## Setup and Usage

1. **Wiring**: Connect the Arduino, encoder, motor driver, and motor according to the circuit diagram.
2. **Upload Code**: Upload the provided code to the Arduino.
3. **Power Up**: Power the system and observe the ball being balanced on the platform.
4. **Adjust Parameters**: If necessary, fine-tune the PID parameters to optimize the balancing performance.
