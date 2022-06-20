#include <Arduino.h>
#include "defines.hpp"
#include "dc_motors.hpp"
#include "wifi.hpp"
#include "april_tag.hpp"

DcMotor mot;

void controlMotorTask(void *argument)
{
  Serial.print("dcMotorControlTask is running on: ");
  Serial.println(xPortGetCoreID());
  DcMotor *mot = (DcMotor *)argument;
  for (;;)
  {
    mot->control();
    yield();
  }
  vTaskDelete(NULL);
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println("start");

  wifiSetup();


  while (1)
  {
    //Serial.println(numDet);
    delay(1000);
  }
  // attach Intterupts:
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), ISR_countA, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), ISR_countB, RISING);
  delay(2000);
  // setup backround task;
  xTaskCreatePinnedToCore(controlMotorTask, "dcMotorControlTask", 10000, &mot, 1, NULL, 1);
  for (size_t i = 0; i < 4; i++)
  {
    mot.driveTurn(11, true);
    delay(500);
  }

  delay(500);
  mot.driveTurn(11, false);
  delay(500);
  mot.driveStraight(150, true);
  delay(500);
  mot.driveStraight(150, false);
  mot.printSteps();
}

void loop()
{
  // put your main code here, to run repeatedly:
  yield();
}