#include <Arduino.h>
#include "defines.hpp"
#include "wifi.hpp"
#include "april_tag.hpp"
#include "car_control.hpp"
#include "stepper_motor.hpp"
#include "ultrasonic.hpp"

#define PIN_TRIGGER 22
#define PIN_ECHO 18

const int SENSOR_MAX_RANGE = 300; // in cm
unsigned long duration;
unsigned int distance;
extern bool telnetConnection;

void setup()
{
  // start serial interface:
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("start");

  // initialize
  stepperMotorsInit();

  // start up AP
  wifiSetup();

  while (telnetConnection == false)
  {
    Serial.println("wait for telnet");
    delay(1000);
  }

  // attachInterrupt(PIN_US0_ECHO, pulse0Echo, CHANGE);
  // setup backround tasks;
  xTaskCreatePinnedToCore(ultrasonicTask, "ultrasonicTask", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(steppersControlTask, "steppersControlTask", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(controlCarTask, "controlCarTask", 10000, NULL, 1, NULL, 1);
}

void loop()
{
  vTaskDelay(0);
}