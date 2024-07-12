#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <AFMotor.h>
#include <Wire.h>

enum Motion {
  UP, DOWN, LEFT, RIGHT, STOP, LEFT_UP, RIGHT_UP, RIGHT_DOWN, LEFT_DOWN
};

// Pins definition
const int in1 = 11;
const int in2 = 10;
const int in3 = 9;
const int in4 = 8;
const int enA = 6;
const int enB = 5;
const int ledBack = 4;
const int ledRight = 13;
const int ledLeft = 12;

// Global variables
int speedPWM = 255;
char command = '\0';
int status = STOP;

// Task handles
TaskHandle_t bluetoothTask;
TaskHandle_t movementTask;
TaskHandle_t speedTask;
TaskHandle_t blinkLedTask;

// Semaphore handle
SemaphoreHandle_t commandSemaphore;

// Queue handle
QueueHandle_t commandQueue;

// Function prototypes
void BLT(void *pvParameters);
void Behavior(void *pvParameters);
void Speed(void *pvParameters);
void BlinkLedSignal(void *pvParameters);
void forward();
void back();
void left();
void right();
void leftUp();
void rightUp();
void rightDown();
void leftDown();
void stop();
void updateLEDs(int status);

void setup() {
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(ledBack, OUTPUT);
  pinMode(ledRight, OUTPUT);
  pinMode(ledLeft, OUTPUT);

  // Create semaphore
  commandSemaphore = xSemaphoreCreateMutex();

  // Create queue
  commandQueue = xQueueCreate(10, sizeof(char));

  // Create tasks
  xTaskCreate(BLT, "BLT", 128, NULL, 1, &bluetoothTask);
  xTaskCreate(Movement, "Movement", 200, NULL, 1, &movementTask);
  xTaskCreate(Speed, "Speed", 128, NULL, 1, &speedTask);
  xTaskCreate(BlinkLedSignal, "BlinkLED", 128, NULL, 1, &blinkLedTask);

  // Start scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty, do not put any code here in FreeRTOS based applications
}

void BLT(void *pvParameters) {
  while (1) {
    if (Serial.available() > 0) {
      char receivedChar = Serial.read();
      xSemaphoreTake(commandSemaphore, portMAX_DELAY);
      xQueueSend(commandQueue, &receivedChar, portMAX_DELAY);
      xSemaphoreGive(commandSemaphore);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void Movement(void *pvParameters) {
  while (1) {
    char receivedCommand;
    if (xQueueReceive(commandQueue, &receivedCommand, portMAX_DELAY) == pdTRUE) {
      command = receivedCommand;
      switch (command) {
        case 'F':
          forward();
          break;
        case 'B':
          back();
          break;
        case 'L':
          left();
          break;
        case 'R':
          right();
          break;
        case 'G':
          leftUp();
          break;
        case 'I':
          rightUp();
          break;
        case 'J':
          rightDown();
          break;
        case 'H':
          leftDown();
          break; 
        default:
          stop();
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void Speed(void *pvParameters) {
  while (1) {
    char receivedCommand;
    if (xQueueReceive(commandQueue, &receivedCommand, portMAX_DELAY) == pdTRUE) {
      command = receivedCommand;
      switch (command) {
        case '0':
          speedPWM = 0;
          break;
        case '1':
          speedPWM = 80;
          break;
        case '2':
          speedPWM = 100;
          break;
        case '3':
          speedPWM = 120;
          break;
        case '4':
          speedPWM = 140;
          break;
        case '5':
          speedPWM = 160;
          break;
        case '6':
          speedPWM = 180;
          break;
        case '7':
          speedPWM = 200;
          break;
        case '8':
          speedPWM = 220;
          break;
        case '9':
          speedPWM = 240;
          break;
        case 'q':
          speedPWM = 255;
          break;
        default:
          break;
      }
      Serial.println(speedPWM);
      analogWrite(enA, speedPWM);
      analogWrite(enB, speedPWM);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void BlinkLedSignal(void *pvParameters) {
  while (1) {
    updateLEDs(status);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void updateLEDs(int status) {
    digitalWrite(ledBack,status == DOWN ? HIGH : LOW);
    digitalWrite(ledLeft, (status == LEFT || status == LEFT_UP || status == LEFT_DOWN) ? HIGH : LOW);
    digitalWrite(ledRight, (status == RIGHT || status == RIGHT_UP || status == RIGHT_DOWN) ? HIGH : LOW);
}

void forward(){
  Serial.println("UP");
  analogWrite(enA, speedPWM); 
  analogWrite(enB, speedPWM); 
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  status = UP;
}

void rightUp(){
  Serial.println("RIGHT_UP");
  analogWrite(enA, speedPWM / 2); 
  analogWrite(enB, speedPWM); 
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  status = RIGHT_UP;
}

void rightDown(){
  Serial.println("RIGHT_DOWN");
  analogWrite(enA, speedPWM / 2); 
  analogWrite(enB, speedPWM); 
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  status = RIGHT_DOWN;
}

void back(){
  Serial.println("DOWN");
  analogWrite(enA, speedPWM); 
  analogWrite(enB, speedPWM); 
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  status = DOWN;
}

void leftUp(){
  Serial.println("LEFT_UP");
  analogWrite(enA, speedPWM); 
  analogWrite(enB, speedPWM / 2); 
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  status = LEFT_UP;
}

void leftDown(){
  Serial.println("LEFT_DOWN");
  analogWrite(enA, speedPWM); 
  analogWrite(enB, speedPWM / 2); 
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  status = LEFT_DOWN;
}

void left(){
  Serial.println("LEFT");
  analogWrite(enA, speedPWM); 
  analogWrite(enB, speedPWM); 
  digitalWrite(in4, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  status = LEFT;
}

void right(){
  Serial.println("RIGHT");
  analogWrite(enA, speedPWM); 
  analogWrite(enB, speedPWM); 
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in2, HIGH);
  digitalWrite(in1, LOW);
  status = RIGHT;
} 

void stop(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  status = STOP;
}
