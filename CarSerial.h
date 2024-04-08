#ifndef _CAR_SERIAL_H_
#define _CAR_SERIAL_H_

#include <Arduino.h>

#define USE_SOFTWARE_SERIAL
#define ESP32_RX 32 // SDA
#define ESP32_TX 33 // SCL

void serial_init(unsigned long baudRate = 9600, const String &name = "RobotCar");

void serial_log(const String &s);

String serial_read();

#endif
