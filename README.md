# AMR Car ESP32

This project is an implementation of an Autonomous Mobile Robot (AMR) using an ESP32 microcontroller. The robot is equipped with motor control, PID management, encoder feedback, and micro-ROS communication.



## ⚙️ Setup

### 📌 Install Required Libraries
Before getting started, install the following libraries in **Arduino IDE**:

| Library | Version |
|---------|---------|
| **PID** | `1.2.0` |
| **ArduinoJson** | `7.3.0` |
| **ESP32Servo** | `3.0.6` |
| **ESP32Encoder** | `0.11.7` |
| **EspSoftwareSerial** | `8.1.0` |
| **Adafruit PWM Servo Driver Library** | `3.0.2` |

---

### 🔹 Install Micro ROS Arduino Library (Humble)
Follow these steps to manually install **Micro ROS Arduino Library (Humble)**:

1. **Download the Library**  
   - Visit the [Micro ROS Arduino Repository](https://github.com/micro-ROS/micro_ros_arduino).  
   - Download the latest **Humble ZIP** package.

2. **Add Library to Arduino IDE**  
   - Open **Arduino IDE**.
   - Navigate to **Sketch → Include Library → Add .ZIP Library**.
   - Select the downloaded `micro_ros_arduino.zip` and click **Open**.

3. **Done! 🎉**  
   - Micro ROS is now installed and ready to use in your Arduino projects.

---

### ⚡ Configure the Pins
- Define the required **GPIO pins** in [`params.hpp`](vscode-file://vscode-app/usr/share/code/resources/app/out/vs/code/electron-sandbox/workbench/workbench.html) based on your hardware setup.

---

### 🔄 Enable Reverse Mode
- If you need **reverse mode**, **uncomment** the following line in `params.hpp`:
  ```cpp
  #define REVERSE_PINS



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

