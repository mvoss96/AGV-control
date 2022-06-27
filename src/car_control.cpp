#include <Arduino.h>
#include "defines.hpp"
#include "car_control.hpp"
#include "stepper_motor.hpp"
#include "april_tag.hpp"
#include "telnet_debug.hpp"

extern unsigned int detectTagCenter;
extern bool udpConnection;
extern bool telnetConnection;
extern uint8_t missionMode;

namespace
{

    bool tagLock = false;
    unsigned long tagTimeoutTimer;
    int followMode = 0;
    bool innerLock = false;
}

enum movement_state
{
    RIGHT,
    LEFT,
    STRAIGHT,
    STRAIGHT_RIGHT,
    STRAIGHT_LEFT
};

void askForMission()
{
    missionMode = missions::NO_MISSION;
    DEBUG_MSG("ask for mission");
    telnet.println("please choose a mission:");
    telnet.println("d-> deliver");
    telnet.println("gg -> get gummy bear");
    telnet.println("gc -> get cotton wool");
    telnet.println("gb -> get ping pong ball");
    while (missionMode == missions::NO_MISSION)
        vTaskDelay(0);
}

void searchForTag()
{
    DEBUG_MSG("start searching for Tag");
    while (1)
    {
        if (udpConnection == false || telnetConnection == false ||
            missionMode == missions::NO_MISSION)
        {
            stepperStop();
            return;
        }
        // wait for detectTag signal
        if (detectTagCenter != 0)
        {
            DEBUG_MSG("tag in view");
            tagLock = true;
            return;
        }
        // make sure robot is turning
        if (stepperIsRunning() == false)
        {
            DEBUG_MSG("start turning");
            stepperStartTurnLeft(5);
        }
        vTaskDelay(0);
    }
}

void followTag()
{
    if (detectTagCenter == 0 || udpConnection == false ||
        telnetConnection == false || missionMode == missions::NO_MISSION)
    {
        return;
    }
    DEBUG_SER_VAR(detectTagCenter);

    if (innerLock)
    {
        if (detectTagCenter < TAG_CENTER - TAG_CENTER_DEADZONE ||
            detectTagCenter > TAG_CENTER + TAG_CENTER_DEADZONE)
        {
            DEBUG_MSG("innerLock lost");
            innerLock = false;
        }
        else if (followMode != movement_state::STRAIGHT || !stepperIsRunning())
        {
            DEBUG_MSG("drive straight");
            followMode = movement_state::STRAIGHT;
            stepperStartStraight(20);
        }
    }
    else
    {
        if (detectTagCenter < TAG_CENTER - TAG_CENTER_DEADZONE_SMALL)
        {
            if (followMode != movement_state::RIGHT || !stepperIsRunning())
            {
                DEBUG_MSG(" turn right");
                followMode = movement_state::RIGHT;
                stepperStartTurnRight(3);
            }
        }
        else if (detectTagCenter > TAG_CENTER + TAG_CENTER_DEADZONE_SMALL)
        {
            if (followMode != movement_state::LEFT || !stepperIsRunning())
            {
                DEBUG_MSG(" turn left");
                followMode = movement_state::LEFT;
                stepperStartTurnLeft(3);
            }
        }
        else
        {
            DEBUG_MSG("innerLock");
            innerLock = true;
        }
    }
}

void waitForUDP()
{
    DEBUG_MSG("carControlTask is waiting for UDP stream");
    while (udpConnection == false)
        vTaskDelay(0);
}

void waitForTelnet()
{
    DEBUG_MSG("carControlTask is waiting for telnet");
    while (telnetConnection == false)
        vTaskDelay(0);
}

void controlCarTask(void *argument)
{
    Serial.print("carControlTask is running on: ");
    Serial.println(xPortGetCoreID());
    // wait for connection
    for (;;)
    {
        if (udpConnection == false)
        {
            tagLock = false;
            stepperStop();
            waitForUDP();
        }
        else if (telnetConnection == false)
        {
            tagLock = false;
            stepperStop();
            waitForTelnet();
        }
        else if (missionMode == missions::NO_MISSION)
        {
            stepperStop();
            askForMission();
        }
        else if (tagLock == false)
        {
            stepperStop();
            searchForTag();
        }
        else if (detectTagCenter > 0)
        {
            tagTimeoutTimer = millis();
            followTag();
        }
        else if (millis() - tagTimeoutTimer > 1000)
        {
            DEBUG_MSG("tagTimeout: tag lock lost");
            tagLock = false;
        }
        vTaskDelay(0);
    }
    Serial.println("carControlTask closed");
    vTaskDelete(NULL);
}