#include "CarSerial.h"

#ifdef USE_SOFTWARE_SERIAL

#include <SoftwareSerial.h>

SoftwareSerial SerialBT(ESP32_RX, ESP32_TX);
#else
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif

void serial_init(unsigned long baudRate, const String &name) {
    Serial.begin(115200);

#ifdef USE_SOFTWARE_SERIAL
    SerialBT.begin(baudRate);
    serial_log("serial_init software");
#else
    SerialBT.begin(name);
    serial_log("serial_init hardware");

#endif

}

void serial_log(const String &s) {
    if (Serial) {
        Serial.println(s);
    }

    if (SerialBT) {
        SerialBT.println(s);
    } else {
        Serial.println("!! BT couldn't write");
    }
}

String serial_read() {
    if (SerialBT.available()) {
        return SerialBT.readStringUntil('\n');
    } else if (Serial.available()) {
        return Serial.readStringUntil('\n');
    }
    return "";
}
