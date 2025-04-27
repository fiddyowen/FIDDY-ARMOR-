# FIDDY ARMOR

**FIDDY ARMOR** (Autonomous Reaching with Machine Object Recognition) is an autonomous robotic system designed for detecting, tracking, and picking up objects using computer vision and a Raspberry Pi-controlled robotic arm.

The project was developed as part of my final-year BEng (Hons) Robotics robitc engineering project at the University of Plymouth, supervised by Dr. Dena Bazazian.

---

## Project Overview

The system integrates:
- Dual camera vision processing using OpenCV
- Conveyor belt control with real-time object detection
- Robotic arm movement using inverse kinematics
- Servo and motor control via PCA9685 PWM driver
- Touchscreen user interface built with Qt Creator
- Full safety features including emergency stop and object hazard detection

The aim was to create a modular, flexible, and fully autonomous pick-and-place system suitable for manufacturing, packaging, and healthcare automation.

---

## Key Features

- **Real-Time Object Detection:** Template matching and HSV color segmentation for robust object recognition.
- **Adaptive Arm Movement:** Real-world scaling and smooth servo transitions using inverse kinematics.
- **Safety Systems:** Emergency stop, hazard detection, and system shutdown protocols.
- **Touchscreen GUI:** User control over conveyor speed, arm smoothness, diagnostics, and emergency stop access.
- **Modular Software Architecture:** Expandable for future features like machine learning and cloud connectivity.

---

## Setup Instructions

1. Hardware:
   - Raspberry Pi 4 (or equivalent)
   - PCA9685 PWM Driver Board
   - servo driver (6)
   - 2 Logitech USB Cameras
   - Custom-built robotic 
   - Custom-built Conveyor belt system
   - Touchscreen display

2. Software:
   - Raspberry Pi OS
   - Qt Creator (for GUI application)
   - OpenCV (for computer vision processing)
   - I2C-dev support for PCA9685 control

3. Running the system:
   - Compile and deploy the Qt project files.
   - Connect cameras and motors to the Raspberry Pi.
   - Launch the main GUI application to start the conveyor and arm control system.

---

## Repository Structure

- `/mainwindow.cpp` – Main menu and UI handling
- `/conveyorcontrol.cpp` – Conveyor system and live object detection
- `/rundiagnosis.cpp` – Diagnostics and emergency stop interface
- `/pca9685.cpp` – Servo motor and conveyor PWM control
- `/settingsdialog.cpp` – Live settings adjustment for conveyor and arm
- `/GUI.pro` – Qt project file for compilation

---

## Project Documentation

Full project documentation, system design, testing results, and evaluation are available in the accompanying final report.

---

## License

This project is licensed for educational purposes. For reuse or further development, please contact [Owen Fiddy](fiddyowen@gmail.com.)

---

