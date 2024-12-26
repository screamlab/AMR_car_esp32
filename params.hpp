#ifndef _PARAMS_HPP_
#define _PARAMS_HPP_

#include <cstdint>

// Define the reverse condition
#define REVERSE_PINS   // Uncomment this line to reverse the pins
#define MOTOR_COUNT 2  // Maximum number of RPM values to collect

// TODO: Refactor this part
extern double targetVelBuffer[MOTOR_COUNT];  // double array to hold the RPM values
extern double currentVelsBuffer[MOTOR_COUNT];
// TODO check encoder resolution
const uint16_t ENCODER_RESOLUTIONS[MOTOR_COUNT] = {330, 350};

/*--------------------------------------------------*/
/*--------------- Pin Definition -------------------*/
/*--------------------------------------------------*/

// I2C
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

// Servo
#define PIN_SERVO 4

// Define the pins
#ifndef REVERSE_PINS
// Motor1 P25->S0 P17->S1 P21->S2
#define PIN_MOTOR_P1 21  // Default: S2
#define PIN_MOTOR_P2 17  // Default: S1
// Motor2 P26->S5 P22->S3 P23->S4
#define PIN_MOTOR2_P1 22  // S3
#define PIN_MOTOR2_P2 23  // S4
// Encoder1 motorA-c1->34 motorA-c2->35 有一個需要相反，因為左右邊
#define PIN_ENCODER_P1 34
#define PIN_ENCODER_P2 35
// Encoder2 motorB-c1->27 motorB-c2->16
#define PIN_ENCODER2_P1 16
#define PIN_ENCODER2_P2 27
#else
#define PIN_MOTOR_P1 17   // Reverse: S1
#define PIN_MOTOR_P2 21   // Reverse: S2
#define PIN_MOTOR2_P1 23  // S4
#define PIN_MOTOR2_P2 22  // S3
#define PIN_ENCODER_P1 35
#define PIN_ENCODER_P2 34
#define PIN_ENCODER2_P1 27
#define PIN_ENCODER2_P2 16
#endif

#define PIN_MOTOR_ENA 25
#define PIN_MOTOR2_ENA 26  // S5

/*--------------------------------------------------*/
/*--------------- Const Definition -----------------*/
/*--------------------------------------------------*/

// Servo
#define SERVO_MIN 60
#define SERVO_MAX 120
#define SERVO_INIT 90
#define SERVO_DELAY 100

// Queue
#define QUEUE_SIZE 10

// Time
#define QUEUE_TIMEOUT 5
#define ENCODER_DELAY 10
#define SERIAL_READ_DELAY 10
#define SERIAL_WRITE_DELAY 10
#define MOTOR_DELAY 10

// Math
#define PI 3.14159

// PID
#define PID_DELAY 100

// Define PID constants
static constexpr double Kp = 5.0;    // 2~8      0.1
static constexpr double Ki = 65.25;  // 0.05~0.5    1~10
static constexpr double Kd = 0.001;

#endif
