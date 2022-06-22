#include <Arduino.h>
#include "defines.hpp"
#include "car_control.hpp"
#include "stepper_motor.hpp"
#include "april_tag.hpp"

extern unsigned int detectTagCenter;
extern bool connection;

namespace
{
    bool tagLock = false;
    unsigned long tagTimeoutTimer;
    int followMode = 0;
}

void searchForTag()
{
    Serial.println("start searching for Tag");
    while (1)
    {
        if (connection == false)
        {
            stepperStop();
            return;
        }
        // wait for detectTag signal
        if (detectTagCenter != 0)
        {
            Serial.println("tag in view");
            tagLock = true;
            return;
        }
        // make sure robot is turning
        if (stepperIsRunning() == false)
        {
            Serial.println("start turning");
            stepperStartTurnLeft();
        }
        vTaskDelay(0);
    }
}

void followTag()
{
    if (detectTagCenter == 0 || connection == false)
    {
        return;
    }
    Serial.print(detectTagCenter);
    if (detectTagCenter < TAG_CENTER - TAG_CENTER_DEADZONE)
    {
        Serial.print(" turn right");
        followMode = 1;
        if (followMode != 1 || !stepperIsRunning())
        {
            stepperStartTurnRight();
        }
    }
    else if (detectTagCenter > TAG_CENTER + TAG_CENTER_DEADZONE)
    {
        followMode = 2;
        Serial.print(" turn left");
        if (followMode != 2 || !stepperIsRunning())
        {
            stepperStartTurnLeft();
        }
    }
    else
    {
        followMode = 3;
        Serial.print(" drive straight");
        if (followMode != 3 || !stepperIsRunning())
        {
            stepperStartStraight();
        }
    }
    Serial.println();
    // delay(500);
}

void controlCarTask(void *argument)
{
    Serial.print("carControlTask is running on: ");
    Serial.println(xPortGetCoreID());
    // wait for connection
    for (;;)
    {
        if (connection)
        {
            // no tag
            if (tagLock == false)
            {
                searchForTag();
            }
            else if (detectTagCenter >= 0)
            {
                tagTimeoutTimer = millis();
                followTag();
            }
            else if (millis() - tagTimeoutTimer > 1000)
            {
                Serial.println("timeout: tag lock lost");
                tagLock = false;
                stepperStop();
            }
        }
        else
        {
            tagLock = false;
            stepperStop();
        }
        vTaskDelay(0);
    }
    Serial.println("carControlTask closed");
    vTaskDelete(NULL);
}