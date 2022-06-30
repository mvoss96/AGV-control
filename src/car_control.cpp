#include <Arduino.h>
#include "defines.hpp"
#include "car_control.hpp"
#include "stepper_motor.hpp"
#include "april_tag.hpp"
#include "telnet_debug.hpp"
#include "ultrasonic.hpp"

extern unsigned int detectTagCenter;
extern unsigned int detectTagSize;
extern bool udpConnection;
extern bool telnetConnection;
extern uint8_t missionMode;
extern bool ultrasonicEnable;
extern double usDistances[3];

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
    STRAIGHT_LEFT,
    BACKWARDS
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

void reposition()
{
    DEBUG_MSG("start reposition");
    delay(1000);
    int REPOSITION_TIMEOUT = 10000;
    int TURN_MIN_TIME = 2000;
    unsigned long repositionTimer = millis();
    unsigned long turnTimer = millis();
    int drivingMode = STRAIGHT;

    while (1)
    {
        if (usDistances[SENSOR_FRONT] > US_NEAR_TRIGGER)
        {
            if (millis() - repositionTimer > REPOSITION_TIMEOUT)
            {
                break;
            }
            if ((millis() - turnTimer > TURN_MIN_TIME && drivingMode != STRAIGHT) || !stepperIsRunning())
            {
                DEBUG_MSG("reposition: drive straight");
                turnTimer = millis();
                drivingMode = STRAIGHT;
                stepperStartStraight(STEPPER_MAX_RPM);
            }
        }
        else
        {
            if (usDistances[SENSOR_LEFT] < US_MIN_TRIGGER && usDistances[SENSOR_RIGHT] < US_MIN_TRIGGER)
            {
                if (drivingMode != BACKWARDS || !stepperIsRunning())
                {
                    DEBUG_MSG("reposition: drive backwards");
                    turnTimer = millis();
                    drivingMode = BACKWARDS;
                    stepperStartBackwards(STEPPER_MAX_RPM);
                }
            }
            else if (usDistances[SENSOR_LEFT] > US_MIN_TRIGGER)
            {
                if (drivingMode != LEFT || !stepperIsRunning())
                {
                    DEBUG_MSG("reposition: turn left");
                    turnTimer = millis();
                    drivingMode = LEFT;
                    stepperStartTurnLeft(STEPPER_TURN_RPM);
                }
            }
            else if (usDistances[SENSOR_RIGHT] > US_MIN_TRIGGER)
            {
                if (drivingMode != RIGHT || !stepperIsRunning())
                {
                    DEBUG_MSG("reposition: turn right");
                    turnTimer = millis();
                    drivingMode = RIGHT;
                    stepperStartTurnRight(STEPPER_TURN_RPM);
                }
            }
        }
        vTaskDelay(0);
    }
    stepperStop();
    DEBUG_MSG(" reposition complete");
}

void searchForTag(bool dir = true)
{
    DEBUG_MSG("start searching");
    unsigned long searchStartTime = millis();
    bool backingOff = false;
    while (1)
    {
        if (millis() - searchStartTime > TAG_SEARCH_TIMEOUT)
        {
            DEBUG_MSG("tag search timeout");
            stepperStop();
            reposition();
            searchStartTime = millis();
        }
        if (udpConnection == false || telnetConnection == false ||
            missionMode == missions::NO_MISSION)
        {
            DEBUG_MSG("stop search due to connection error");
            stepperStop();
            return;
        }
        // if tag detected
        if (detectTagCenter != 0)
        {
            DEBUG_MSG("tag in view");
            tagLock = true;
            return;
        }

        // check ultrasonic sensors
        if (usDistances[SENSOR_FRONT] < US_MIN_TRIGGER)
        {
            if (!stepperIsRunning() || backingOff == false)
            {
                DEBUG_MSG("obstacle in front: drive backwards");
                backingOff = true;
                stepperStartBackwards(STEPPER_MAX_RPM);
                vTaskDelay(100);
            }
        }
        else if (usDistances[SENSOR_LEFT] < US_MIN_TRIGGER && usDistances[SENSOR_RIGHT] < US_MIN_TRIGGER)
        {
            DEBUG_MSG("obstacle on both sides while turning");
            stepperStop();
            reposition();
        }
        else
        {
            if (usDistances[(dir) ? SENSOR_LEFT : SENSOR_RIGHT] < US_MIN_TRIGGER)
            {
                DEBUG_MSG("obstacle while turning: changing direction");
                stepperStop();
                dir = !dir;
            }

            // make sure robot is turning
            if (stepperIsRunning() == false || backingOff == true)
            {
                backingOff = false;
                if (dir)
                {
                    DEBUG_MSG("start turning left");
                    stepperStartTurnLeft(STEPPER_TURN_RPM);
                }
                else
                {
                    DEBUG_MSG("start turning right");
                    stepperStartTurnRight(STEPPER_TURN_RPM);
                }
            }
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
            if (detectTagSize < TAG_CLOSE_SIZE)
            {
                DEBUG_MSG("drive straight");
                followMode = movement_state::STRAIGHT;
                stepperStartStraight(STEPPER_MAX_RPM);
            }
            else
            {
                DEBUG_MSG("Beacon reached");
                stepperStop();
                vTaskDelay(100);
            }
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
                stepperStartTurnRight(STEPPER_SLOW_TURN_RPM);
            }
        }
        else if (detectTagCenter > TAG_CENTER + TAG_CENTER_DEADZONE_SMALL)
        {
            if (followMode != movement_state::LEFT || !stepperIsRunning())
            {
                DEBUG_MSG(" turn left");
                followMode = movement_state::LEFT;
                stepperStartTurnLeft(STEPPER_SLOW_TURN_RPM);
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