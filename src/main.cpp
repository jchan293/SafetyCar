/** 
 * @file main.cpp
 * by Pranav Peddinti and Jorlly Chang
 * @brief This file contains the code for the Safety Car project.
 * This code is designed to run on an ESP32 microcontroller and control a small car.
 * It includes motor control functions to move the car forward, backward, and stop.
 * It also includes a web server to control the car remotely.
 * The code measures data from a ToF sensor to detect objects in front of the car.
 * If an object is detected within 1 meter, the car will automatically stop.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

// Define motor control pins
#define MOTOR_IN1 26
#define MOTOR_IN2 25

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
  #define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

// Initialize the VL53L4CX sensor
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, A1);

// WiFi credentials
const char* ssid = "ESP32_AP";
const char* password = "12345678";

// Create a web server object
WebServer server(80);



/**
 * @brief Moves the car forward.
 * 
 * Sets MOTOR_IN1 to LOW and MOTOR_IN2 to HIGH to move the motor forward.
 * Outputs a message to the Serial Monitor indicating the motor is running forward.
 */
void moveForward() {
  digitalWrite(MOTOR_IN1, LOW);
  analogWrite(MOTOR_IN2, 255);
  Serial.println("Motor running forward...");
}

/**
 * @brief Moves the car backward.
 * 
 * Sets MOTOR_IN1 to HIGH and MOTOR_IN2 to LOW to move the motor backward.
 * Outputs a message to the Serial Monitor indicating the motor is running backward.
 */
void moveBackward() {
  digitalWrite(MOTOR_IN1, HIGH);
  analogWrite(MOTOR_IN2, 0);
  Serial.println("Motor running backward...");
}

/**
 * @brief Stops the motor.
 * 
 * Sets both MOTOR_IN1 and MOTOR_IN2 to LOW to stop the motor.
 * Outputs a message to the Serial Monitor indicating the motor is stopped.
 * 
 * @note This function was shown during the demonstration and cuts power to the motor,
 * resulting in the car coasting to a stop.
 */
void stopMotor() {
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, 0);
  Serial.println("Motor stopped");
}

/**
 * @brief Brakes the motor.
 * 
 * Sets both MOTOR_IN1 and MOTOR_IN2 to HIGH to brake the motor.
 * Outputs a message to the Serial Monitor indicating the motor is braking.
 * 
 * @note This function is a work in progress and was not shown during the demonstration.
 */
void brakeMotor() {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, HIGH); // Both pins set to HIGH to brake the motor
  Serial.println("Motor braking...");
}

/**
 * @brief Handles web requests for the gear selector.
 * 
 * This section of the code provides the functionality for a gear selector
 * for the car, which works by pressing the corresponding links on the webpage.
 */
void handleRoot() {
  server.send(200, "text/html", "<h1>Gear Selector</h1><p><a href=\"/forward\">Drive</a></p><p><a href=\"/backward\">Reverse</a></p><p><a href=\"/stop\">Park</a></p>");
}

void handleForward() {
  moveForward();
  server.send(200, "text/html", "<h1>In Drive</h1><p><a href=\"/\">Back</a></p>");
}

void handleBackward() {
  moveBackward();
  server.send(200, "text/html", "<h1>Reversing!</h1><p><a href=\"/\">Back</a></p>");
}

void handleStop() {
  stopMotor();
  server.send(200, "text/html", "<h1>Parked</h1><p><a href=\"/\">Back</a></p>");
}

/**
 * @brief Initializes the system and sets up the web server.
 * 
 * This function performs the following tasks:
 * - Initializes the motor control pins as outputs.
 * - Initializes the LED pin as an output.
 * - Initializes serial communication for output.
 * - Initializes the I2C bus.
 * - Configures and initializes the VL53L4CX satellite component.
 * - Sets up the ESP32 as a WiFi access point.
 * - Sets up the web server routes for handling motor control commands.
 */
void setup() {
  // Initialize the motor control pins as outputs
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(LedPin, OUTPUT);

  // Initialize serial for output
  Serial.begin(9600);  // 115200 was not working
  Serial.println("Starting...");

  // Initialize I2C bus
  DEV_I2C.begin();

  // Configure VL53L4CX satellite component
  sensor_vl53l4cx_sat.begin();
  sensor_vl53l4cx_sat.VL53L4CX_Off();
  sensor_vl53l4cx_sat.InitSensor(0x12);
  sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();

  // Set up WiFi
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Set up web server routes
  server.on("/", handleRoot);
  server.on("/forward", handleForward);
  server.on("/backward", handleBackward);
  server.on("/stop", handleStop);
  server.begin();
  Serial.println("Web server started");
}

/**
 * @brief Main loop function.
 * 
 * This function performs the following tasks in a continuous loop:
 * - Handles incoming client requests for the web server.
 * - Reads data from the VL53L4CX sensor.
 * - Turns on the LED while reading sensor data.
 * - Outputs the sensor data to the Serial Monitor.
 * - Clears the sensor interrupt and starts a new measurement.
 * - Checks if the detected object is within 1 meter and applies the brakes if necessary.
 * - Turns off the LED after processing the sensor data.
 */
void loop() {
  server.handleClient();

  VL53L4CX_MultiRangingData_t MultiRangingData;
  VL53L4CX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
  uint8_t NewDataReady = 0;
  int no_of_object_found = 0, j;
  char report[64];
  int status;

  do {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  } while (!NewDataReady);

  // Led on
  digitalWrite(LedPin, HIGH);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(pMultiRangingData);
    no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
    snprintf(report, sizeof(report), "VL53L4CX Satellite: Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
    Serial.print(report);
    for (j = 0; j < no_of_object_found; j++) {
      if (j != 0) {
        Serial.print("\r\n                               ");
      }
      Serial.print("status=");
      Serial.print(pMultiRangingData->RangeData[j].RangeStatus);
      Serial.print(", D=");
      Serial.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
      Serial.print("mm");
    }
    Serial.println("");
    if (status == 0) {
      status = sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }

    // Check if the distance is less than or equal to 1 meter (1000 mm)
    if (MultiRangingData.RangeData[0].RangeMilliMeter <= 1000) {
      stopMotor(); // Apply the "brakes" when the distance is less than or equal to 1 meter
    }
  }

  digitalWrite(LedPin, LOW);
}
