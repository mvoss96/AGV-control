#include <Arduino.h>
#include "defines.hpp"
#include "wifi.hpp"
#include "april_tag.hpp"
#include "car_control.hpp"
#include "stepper_motor.hpp"

void setup()
{
  // start serial interface:
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("start");
  stepperMotorsInit();

  delay(3000);

  // start up networking
  wifiSetup();

  // setup backround tasks;
  xTaskCreatePinnedToCore(steppersControlTask, "steppersControlTask", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(controlCarTask, "controlCarTask", 10000, NULL, 1, NULL, 1);

}

void loop()
{
  yield();
}