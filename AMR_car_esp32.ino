/*--------------------------------------------------*/
/*---------------------- Include Library -----------*/
/*--------------------------------------------------*/
#include <ESP32Encoder.h>
#include <InterruptEncoder.h>
#include <ArduinoJson.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
// #include "serial.h"
#define USE_SOFTWARE_SERIAL

#include "CarSerial.h"
#include "EncoderManager.h"
#include "PIDManager.h"
#include <PID_v1.h>

// micro ros
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>
#include <std_msgs/msg/int64_multi_array.h>

std_msgs__msg__Float32MultiArray msg;
// std_msgs__msg__Int64MultiArray msg;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_node_options_t node_ops;
bool micro_ros_init_successful;
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\


void error_loop(){
  while(1){
    delay(100);
  }
}

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;


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

// Motor1 P25->S0 P17->S1 P21->S2
#define PIN_MOTOR_P1 21 // S2
#define PIN_MOTOR_P2 17 // S1
#define PIN_MOTOR_ENA 25
// Motor2 P26->S5 P22->S3 P23->S4
#define PIN_MOTOR2_P1 22  // S3
#define PIN_MOTOR2_P2 23  // S4
#define PIN_MOTOR2_ENA 26 // S5

// Encoder1 motorA-c1->34 motorA-c2->35 有一個需要相反，因為左右邊
#define PIN_ENCODER_P1 34
#define PIN_ENCODER_P2 35

// Encoder2 motorB-c1->27 motorB-c2->16
#define PIN_ENCODER2_P1 16
#define PIN_ENCODER2_P2 27

#define MOTOR_COUNT 2 // Maximum number of RPM values to collect

double targetVelBuffer[MOTOR_COUNT]; // double array to hold the RPM values
double currentVelsBuffer[MOTOR_COUNT];
// TODO check encoder resolution
const uint16_t ENCODER_RESOLUTIONS[MOTOR_COUNT] = {330, 350};
/*--------------------------------------------------*/
/*--------------- Const Definition -----------------*/
/*--------------------------------------------------*/

// Servo
#define SERVO_MIN 60
#define SERVO_MAX 120
#define SERVO_INIT 90
#define SERVO_DELAY 100

// PID
#define PID_DELAY 100

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

/*--------------------------------------------------*/
/*--------------- Global variable Definition -------*/
/*--------------------------------------------------*/

// Servo
Servo myservo;
uint8_t servo_direction = 90;
uint8_t servo_bias = 10;


// Define PID constants
double Kp = 5.0; //2~8      0.1
double Ki = 65.25; //0.05~0.5    1~10
double Kd = 0.001;
PIDManager pid_manager(Kp, Ki, Kd, MOTOR_COUNT);

// Time Variables
unsigned long prevTime = 0;
float deltaTime = 0;

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

// void micro_ros_init();
bool create_entities();
/*--------------------------------------------------*/
/*--------------- Setup ----------------------------*/
/*--------------------------------------------------*/
void setup() {
    Serial.begin(115200);
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
    

    // initial ros node and subscriber
    // micro_ros_init();
    

    // Task                 task handle       task name         stack size
    //                      paremeter         priority          task_handle, core
    xTaskCreatePinnedToCore(TaskServo, "TaskServo", 1024,
                            NULL, 1, NULL, 0);
    delay(100);
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
                            NULL, 1, NULL, 1);
    delay(100);
    xTaskCreatePinnedToCore(MicroROSWheel, "MicroROSWheel", 4096,
                            NULL, 3, NULL, 0);
    delay(100);
}

void loop() {
    // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskSerialRead(void *pvParameters) {
    serial_log("TaskSerialRead() running on core ");
    String data;
    StaticJsonDocument<200> doc;

    for (;;) {
        data = serial_read(); // 從串口接收資料
        if (data != "") {
            DeserializationError error = deserializeJson(doc, data);
            if (error) {
                serial_log("deserializeJson() failed: ");
                serial_log("Received data: ");
                serial_log(data);
                continue;
            }
            

            // 檢查 JSON 中是否有 "command" 並且值為 "restart"
            if (doc.containsKey("command") && doc["command"] == "restart") {
                // digitalWrite(LED_PIN, LOW);
                serial_log("Restart command received. Restarting ESP32...");
                delay(1000); // 給系統一秒時間處理剩餘工作
                ESP.restart(); // 軟體重啟 ESP32
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
        vTaskDelay(SERIAL_READ_DELAY / portTICK_PERIOD_MS); // 延遲穩定性
    }
}


void MicroROSWheel(void *pvParameters){
  set_microros_transports();
  state = WAITING_AGENT;
  while(1){
    switch (state) {
      case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        // if (state == WAITING_AGENT){
        //   ESP.restart();
        // }
        break;
      case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) {
          destroy_entities();
        };
        break;
      case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
      case AGENT_DISCONNECTED:
        destroy_entities();
        state = WAITING_AGENT;
        break;
      default:
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  
    
}

bool create_entities()
{
  
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 1);
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
  &subscriber,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  "car_C_rear_wheel"));

  msg.data.capacity = 10; 
  msg.data.size = 0;
  msg.data.data = (float*) malloc(msg.data.capacity * sizeof(float));
  msg.layout.dim.capacity = 10;
  msg.layout.dim.size = 0;
  msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
  for(size_t i = 0; i < msg.layout.dim.capacity; i++){
      msg.layout.dim.data[i].label.capacity = 10;
      msg.layout.dim.data[i].label.size = 0;
      msg.layout.dim.data[i].label.data = (char*) malloc(msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  return true;
}
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  targetVelBuffer[0] = msg->data.data[0];
  targetVelBuffer[1] = msg->data.data[1];
  // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

void TaskSerialWrite(void *pvParameters) {
    serial_log("TaskSerialWrite() running on core ");
    serial_log(String(xPortGetCoreID()));

    //TODO refactor init_encoder to other function
    // use pin 19 and 18 for the first encoder
    uint8_t pins[MOTOR_COUNT][2] = {
            {PIN_ENCODER_P1,  PIN_ENCODER_P2},  // Encoder 1 pins
            {PIN_ENCODER2_P1, PIN_ENCODER2_P2} // Encoder 2 pins
    };

    EncoderManager encoderManager(pins, MOTOR_COUNT, ENCODER_RESOLUTIONS);

    for (;;) {
        // Create a JSON document
        DynamicJsonDocument doc(200); // Adjust the size based on your data size

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

        vTaskDelay(SERIAL_WRITE_DELAY / portTICK_PERIOD_MS); // one tick delay (100ms) in between reads for stability
    }
}

void TaskServo(void *pvParameters) {
    serial_log("TaskServo() running on core ");
    serial_log(String(xPortGetCoreID()));
    int32_t direction = 0;
    servo_init(); // myservo.attach(PIN_SERVO,SERVO_MIN,SERVO_MAX);
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
    serial_log("TaskPID() running on core ");
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
    myservo.attach(PIN_SERVO); // 設置舵機控制腳位
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

    serial_log("Motor pin setup");
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

void motor_execute(uint8_t num, int16_t vel) { // 副程式  前進
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
        digitalWrite(motor_p1, LOW);  // control the motor's direction in clockwise
        digitalWrite(motor_p2, HIGH); // control the motor's direction in clockwise
        analogWrite(motor_en, vel);
    } else {
        digitalWrite(motor_p1, HIGH); // control the motor's direction in clockwise
        digitalWrite(motor_p2, LOW);  // control the motor's direction in clockwise
        analogWrite(motor_en, -vel);
    }
}
void destroy_entities() {
    targetVelBuffer[0] = 0.0;
    targetVelBuffer[1] = 0.0;
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    // 銷毀其他實體
    rcl_subscription_fini(&subscriber, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    // 手動釋放動態分配的內存
    if (msg.data.data != NULL) {
        free(msg.data.data);
        msg.data.data = NULL;  // 避免懸空指針
    }

    if (msg.layout.dim.data != NULL) {
        // 如果有嵌套的分配，先釋放內部
        for (size_t i = 0; i < msg.layout.dim.capacity; i++) {
            if (msg.layout.dim.data[i].label.data != NULL) {
                free(msg.layout.dim.data[i].label.data);
                msg.layout.dim.data[i].label.data = NULL;
            }
        }
        // 再釋放外層的指針
        free(msg.layout.dim.data);
        msg.layout.dim.data = NULL;
    }
}