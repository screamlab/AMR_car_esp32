#include "EncoderManager.h"

EncoderManager::EncoderManager(uint8_t pins[][2], const uint8_t encoderCount, const uint16_t encoder_resolutions[]) {
    this->encoderCount = encoderCount;
    this->encoderResolutions = encoder_resolutions;

    // Initialize the encoders
    encoders = new ESP32Encoder[encoderCount];
    prevCounts = new int[encoderCount];
    for (uint8_t i = 0; i < encoderCount; i++) {
        //  一圈 1000
        encoders[i].attachSingleEdge(pins[i][0], pins[i][1]);
        //  一圈 2000
        // encoders[i].attachHalfQuad(pins[i][0], pins[i][1]);
        //  一圈 4000
        // encoders[i].attachFullQuad(pins[i][0], pins[i][1]);

        encoders[i].clearCount();
        encoders[i].setCount(0);
        prevCounts[i] = 0;
    }
    prevTime = millis();
}

int *EncoderManager::getCounts() {
    static int counts[MAX_ENCODERS]; // Static array to store encoder counts

    for (int i = 0; i < encoderCount; i++) {
        counts[i] = encoders[i].getCount();
    }

    return counts;
}

float *EncoderManager::getAngularVel() {
    static float vels[MAX_ENCODERS]; // Static array to store vel values

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0;
    int *counts = getCounts();
    for (int i = 0; i < this->encoderCount; i++) {
        int encoderDiff = counts[i] - prevCounts[i];
        if (encoderDiff == 0) {
            vels[i] = 0;
            continue;
        }
        // rad/s
        vels[i] = (2 * 3.14159 * encoderDiff / this->encoderResolutions[i]) / deltaTime;
        // rev per min
        // vels[i] = ( 60.0 * encoderDiff / encoderResolutions[i]) / deltaTime;
        prevCounts[i] = counts[i];
        // Serial.print("CurrentTime: ");
        // Serial.print(currentTime);
        // Serial.print(" PrevTime: ");
        // Serial.print(prevTime);
        // Serial.print(" DeltaTime: ");
        // Serial.print(deltaTime);
        // Serial.print(" Encoder ");
        // Serial.print(i);
        // Serial.print(", Diff: ");
        // Serial.print(encoderDiff);
        // Serial.print(", vel: ");
        // Serial.println(vels[i]);
    }

    prevTime = currentTime;

    return vels;
}
