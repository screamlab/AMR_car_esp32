/*--------------------------------------------------*/
/*---------------------- Include Library -----------*/
/*--------------------------------------------------*/
#include <ArduinoJson.h>
#include <ESP32Encoder.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <InterruptEncoder.h>
// #include "serial.h"
#define USE_SOFTWARE_SERIAL

#include <PID_v1.h>

#include "CarSerial.h"
#include "EncoderManager.h"
#include "PIDManager.h"

// #include "params.hpp"
#include "micro_ros.hpp"

/*--------------------------------------------------*/
/*--------------- Global variable Definition -------*/
/*--------------------------------------------------*/

// Servo
Servo myservo;

// Define PID constants
PIDManager pid_manager(Kp, Ki, Kd, MOTOR_COUNT);

// Queue
QueueHandle_t encoder_queue;
QueueHandle_t motor_queue;
QueueHandle_t servo_queue;

/*--------------------------------------------------*/
/*--------------- Function Definition---------------*/
/*--------------------------------------------------*/

void TaskSerialRead(void *pvParameters);

void TaskSerialWrite(void *pvParameters);

void TaskTestPID(void *pvParameters);

void TaskServo(void *pvParameters);

void TaskPID(void *pvParameters);

// void TaskImu(void *pvParameters);
void backward(int vel);

void forward(int vel);

void motor_execute(int vel);

void motor_execute(uint8_t num, int16_t vel);

void queue_init();

void servo_init();

void motor_init();

/*--------------------------------------------------*/
/*--------------- Setup ----------------------------*/
/*--------------------------------------------------*/
void setup() {
    Serial.begin(921600);
    // Serial
    delay(1000);
    // serial_init(9600,"Robot_driver_03");

    serial_init();
    delay(50);

    // Servo
    servo_init();
    delay(50);

    // Motor
    motor_init();

    // Queue
    queue_init();

    // micro ros

    // Task                 task handle       task name         stack size
    //                      paremeter         priority          task_handle, core
    // xTaskCreatePinnedToCore(TaskServo, "TaskServo", 1024,
    //                         NULL, 1, NULL, 0);
    // delay(100);
    // xTaskCreatePinnedToCore(TaskTestPID, "TaskTestPID", 1024,
    //                         NULL, 2, NULL, 1);
    // delay(100);
    // xTaskCreatePinnedToCore(TaskSerialRead, "TaskSerialRead", 1024,
    //                         NULL, 4, NULL, 1);
    // delay(100);
    xTaskCreatePinnedToCore(TaskSerialWrite, "TaskSerialWrite", 2560,
                            NULL, 3, NULL, 0);
    delay(100);
    xTaskCreatePinnedToCore(TaskPID, "TaskPID", 1024,
                            NULL, 1, NULL, 0);
    delay(100);
    xTaskCreatePinnedToCore(MicroROSWheel, "MicroROSWheel", 4096,
                            NULL, 1, NULL, 1);
    delay(100);
}

void loop() {
    // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskSerialRead(void *pvParameters) {
    // serial_log("TaskSerialRead() running on core ");
    String data;
    StaticJsonDocument<200> doc;

    for (;;) {
        data = serial_read();  // 從串口接收資料
        if (data != "") {
            DeserializationError error = deserializeJson(doc, data);
            if (error) {
                // serial_log("deserializeJson() failed: ");
                // serial_log("Received data: ");
                // serial_log(data);
                continue;
            }

            // 檢查 JSON 中是否有 "command" 並且值為 "restart"
            if (doc.containsKey("command") && doc["command"] == "restart") {
                // digitalWrite(LED_PIN, LOW);
                // serial_log("Restart command received. Restarting ESP32...");
                delay(1000);    // 給系統一秒時間處理剩餘工作
                ESP.restart();  // 軟體重啟 ESP32
            }

            // 如果 JSON 有 "target_vel"，則解析並存入 targetVelBuffer
            // if (doc.containsKey("target_vel")) {
            //     JsonArray targetVelArray = doc["target_vel"].as<JsonArray>();
            //     uint8_t motor_i = 0;
            //     for (const auto &value : targetVelArray) {
            //         float vel = value.as<float>();
            //         if (motor_i < MOTOR_COUNT) {
            //             targetVelBuffer[motor_i] = vel;
            //             motor_i++;
            //         }
            //     }
            // }
        }

        // 背景例行處理
        vTaskDelay(SERIAL_READ_DELAY / portTICK_PERIOD_MS);  // 延遲穩定性
    }
}

void TaskSerialWrite(void *pvParameters) {
    // serial_log("TaskSerialWrite() running on core ");
    // serial_log(String(xPortGetCoreID()));

    // TODO refactor init_encoder to other function
    //  use pin 19 and 18 for the first encoder
    uint8_t pins[MOTOR_COUNT][2] = {
        {PIN_ENCODER_P1, PIN_ENCODER_P2},   // Encoder 1 pins
        {PIN_ENCODER2_P1, PIN_ENCODER2_P2}  // Encoder 2 pins
    };

    EncoderManager encoderManager(pins, MOTOR_COUNT, ENCODER_RESOLUTIONS);

    for (;;) {
        // Create a JSON document
        DynamicJsonDocument doc(200);  // Adjust the size based on your data size

        int *counts = encoderManager.getCounts();
        float *vels = encoderManager.getAngularVel();

        // compute radian per second
        // Create the current vel array
        JsonArray currentRPSArray = doc.createNestedArray("vels");
        // Create the encoder array
        JsonArray encoderArray = doc.createNestedArray("encoders");
        for (int i = 0; i < MOTOR_COUNT; i++) {
            currentRPSArray.add(vels[i]);
            currentVelsBuffer[i] = vels[i];
            encoderArray.add(counts[i]);
            // serial_log(String("Motor ") + String(i+1) + ": Current RPM = " + String(vels[i] * 60.0 / (2 * PI))); // 將角速度轉換為 RPM
        }

        // Add the direction value
        doc["direction"] = 180 - myservo.read();
        // doc["mem"] = ESP.getFreeHeap();

        // Serialize the JSON document to a string
        String jsonString;
        serializeJson(doc, jsonString);

        // Print the JSON string to the Serial monitor
        // serial_log(jsonString);

        vTaskDelay(SERIAL_WRITE_DELAY / portTICK_PERIOD_MS);  // one tick delay (100ms) in between reads for stability
    }
}

void TaskServo(void *pvParameters) {
    serial_log("TaskServo() running on core ");
    serial_log(String(xPortGetCoreID()));
    int32_t direction = 0;
    servo_init();  // myservo.attach(PIN_SERVO,SERVO_MIN,SERVO_MAX);
    for (;;) {
        // read servo angle from queue
        if (xQueueReceive(servo_queue, &direction, QUEUE_TIMEOUT) == pdTRUE) {
            // direction = constrain(direction, SERVO_MIN, SERVO_MAX);
            myservo.write(180 - direction);
        }
        vTaskDelay(SERVO_DELAY / portTICK_PERIOD_MS);
    }
}

void TaskTestPID(void *pvParameters) {
    serial_log("TaskMotor() running on core ");
    serial_log(String(xPortGetCoreID()));
    int32_t target = 0;
    bool increase = true;
    for (;;) {
        if (increase) {
            target += 20;
        } else {
            target -= 20;
        }
        targetVelBuffer[0] = target;
        targetVelBuffer[1] = target;
        if (target > 110) {
            increase = false;
        } else if (target < -110) {
            increase = true;
        }
        vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
}

void TaskPID(void *pvParameters) {
    // serial_log("TaskPID() running on core ");
    serial_log(String(xPortGetCoreID()));
    const TickType_t xDelay = pdMS_TO_TICKS(PID_DELAY);
    static double outputs_buff[MOTOR_COUNT];
    for (;;) {
        pid_manager.setSetpoints(targetVelBuffer);
        double *outputs_buff = pid_manager.compute(currentVelsBuffer);
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (targetVelBuffer[i] >= 0) {
                motor_execute(i + 1, outputs_buff[i]);
            } else {
                motor_execute(i + 1, -outputs_buff[i]);
            }

            // motor_execute(2,outputs_buff[1]);
            //   Serial.print("motor:");
            // Serial.print(i+1);

            //   Serial.print("\tTarget:");
            // Serial.print(targetVelBuffer[0]);
            // Serial.print("\tCurrent:");
            // Serial.print(currentVelsBuffer[0]);
            // Serial.print("\tPWM:");
            // Serial.println(outputs_buff[0]);
        }

        // Print the input, output, and setpoint values

        vTaskDelay(xDelay);
    }
}
/*--------------------------------------------------*/
/*----------- Function Implementation --------------*/
/*--------------------------------------------------*/

void servo_init() {
    myservo.attach(PIN_SERVO);  // 設置舵機控制腳位
    if (myservo.attached()) {
        serial_log("Servo is attached");
        myservo.write(SERVO_INIT);
    } else {
        serial_log("Servo is not attached");
    }
    // Serial.println("Servo init");
}

void motor_init() {
    pinMode(PIN_MOTOR_P1, OUTPUT);
    pinMode(PIN_MOTOR_P2, OUTPUT);
    pinMode(PIN_MOTOR_ENA, OUTPUT);
    pinMode(PIN_MOTOR2_P1, OUTPUT);
    pinMode(PIN_MOTOR2_P2, OUTPUT);
    pinMode(PIN_MOTOR2_ENA, OUTPUT);
    pinMode(PIN_ENCODER_P1, INPUT_PULLUP);
    pinMode(PIN_ENCODER_P2, INPUT);

    pinMode(PIN_ENCODER2_P1, INPUT_PULLUP);
    pinMode(PIN_ENCODER2_P1, INPUT);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        targetVelBuffer[i] = 0.0;
        currentVelsBuffer[i] = 0.0;
    }

    // serial_log("Motor pin setup");
}

void queue_init() {
    encoder_queue = xQueueCreate(QUEUE_SIZE, sizeof(int));
    // imu_queue = xQueueCreate(QUEUE_SIZE, 200);
    servo_queue = xQueueCreate(QUEUE_SIZE, sizeof(int));
    motor_queue = xQueueCreate(QUEUE_SIZE, sizeof(int));

    if (encoder_queue == NULL || servo_queue == NULL || motor_queue == NULL) {
        serial_log("Error creating the queue");
    }
}

void motor_execute(uint8_t num, int16_t vel) {  // 副程式  前進
    uint8_t motor_p1, motor_p2, motor_en;
    if (num == 1) {
        motor_p1 = PIN_MOTOR_P1;
        motor_p2 = PIN_MOTOR_P2;
        motor_en = PIN_MOTOR_ENA;
    } else if (num == 2) {
        motor_p1 = PIN_MOTOR2_P1;
        motor_p2 = PIN_MOTOR2_P2;
        motor_en = PIN_MOTOR2_ENA;
    } else {
        return;
    }

    if (vel > 0) {
        digitalWrite(motor_p1, LOW);   // control the motor's direction in clockwise
        digitalWrite(motor_p2, HIGH);  // control the motor's direction in clockwise
        analogWrite(motor_en, vel);
    } else {
        digitalWrite(motor_p1, HIGH);  // control the motor's direction in clockwise
        digitalWrite(motor_p2, LOW);   // control the motor's direction in clockwise
        analogWrite(motor_en, -vel);
    }
}
