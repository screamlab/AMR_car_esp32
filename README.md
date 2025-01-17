# AMR Car ESP32

This project is an implementation of an Autonomous Mobile Robot (AMR) using an ESP32 microcontroller. The robot is equipped with motor control, PID management, encoder feedback, and micro-ROS communication.



## Setup

1. Install the required libraries:
   - PID
   - ~~ArduinoJson~~
   - ESP32Servo
   - ESP32Encoder
   - EspSoftwareSerial
   - Adafruit PWM Servo Driver Library
   - [Micro ROS Arduino Library](https://github.com/micro-ROS/micro_ros_arduino)
2. Configure the pins:
   - Define the pins in [params.hpp](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html) as per your hardware setup.
3. **Reverse Mode**:
   - To enable reverse mode, uncomment the `#define REVERSE_PINS` line in `params.hpp`.



## Usage

1. **Serial Communication**:
   - The serial communication is handled by [CarSerial.cpp](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html) and [CarSerial.h](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html). It supports both software and hardware serial communication.

2. **PID Management**:
   - The PID control for the motors is managed by [PIDManager.h](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html).

3. **Encoder Management**:
   - The encoder feedback is managed by [EncoderManager.cpp](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html) and [EncoderManager.h](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html).

4. **Micro-ROS Communication**:

   - Visit the Micro ROS Arduino [release page](https://github.com/micro-ROS/micro_ros_arduino/releases) and download the ZIP file corresponding to your ROS2 version.

   - The micro-ROS communication is implemented in [micro_ros.cpp](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html) and [micro_ros.hpp](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html).

5. **Tasks**:
   - The main tasks are defined in `AMR_car_esp32.ino`:
     - [TaskSerialRead](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html)
     - [TaskSerialWrite](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html)
     - [TaskTestPID](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html)
     - [TaskServo](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html)
     - [TaskPID](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html)



## Reverse

In `params.hpp`, there is the following definition:

```c
#define REVERSE_PINS
```

To enable reverse mode, declare this variable. Otherwise, comment out this variable.

