# STM32-IMU-Driven-Adaptive-Lighting-Controller

This repository contains the source code and documentation for **STM32 IMU Driven Adaptive Lighting-Controller**, where an STM32 microcontroller interfaces with an MPU6050 IMU sensor to compute roll angle using a Kalman filter and dynamically control LED brightness via PWM.  
The project demonstrates real-time embedded system design using STM32 HAL, FreeRTOS task scheduling, and custom sensor drivers.

<p align="center">
  <img src="https://github.com/OceanBhatnagar/STM32-IMU-Driven-Adaptive-Lighting-Controller/blob/main/Project%20Photograph/AT%20LEFT%20ORIENTATION.jpeg?raw=true" width="600" title="Project Hardware Setup">
</p>

## Features  
- **MPU6050 IMU Interface**: Reads accelerometer and gyroscope data over I2C for motion sensing.  
- **Kalman Filter Implementation**: Sensor fusion algorithm to accurately estimate roll angle with noise reduction.  
- **PWM LED Control**: Roll angle mapped to PWM duty cycle to vary brightness of two LEDs.  
- **FreeRTOS Integration**: Separate tasks for sensor acquisition, filtering, and LED control for real-time performance.  
- **Modular Firmware Architecture**: Layered design separating drivers, middleware, and application logic.

## Components Used  
- **STM32 Nucleo Development Board**  
- **MPU6050 IMU Sensor**  
- **2 LEDs**  
- **Resistors**  
- **Breadboard** and **Jumper Wires**

## Tools Used  
- **STM32CubeIDE**: Firmware development and debugging  
- **STM32CubeMX**: Peripheral configuration and initialization code generation  
- **FreeRTOS**: Real-time task scheduling and system management
- **STM32CubeMonitor**: Real Time Visualization of Roll Angle and IMU Data.    

## Project Structure  
- **CODE/Drivers**: Peripheral interface files  
- **CODE/Middleware**: Kalman filter implementation and RTOS configuration
- **CODE/BSP**:MPU6050 BSP and Kalman Filter Driver Files    
- **CODE/Src & CODE/Inc**: Application logic for angle calculation and LED brightness control  
- **Project Photograph**: Demonstrates hardware setup and real-time testing
