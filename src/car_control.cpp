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
extern bool ultrasonicStarted;
extern double usDistances[NUM_SENSORS];

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

double sensor_front()
{
    return usDistances[SENSOR_FRONTC];
}

double sensor_front_all()
{
    return min(usDistances[SENSOR_FRONTL], min(usDistances[SENSOR_FRONTC], usDistances[SENSOR_FRONTR]));
}

double sensor_front_out()
{
    return min(usDistances[SENSOR_FRONTL], usDistances[SENSOR_FRONTR]);
}

double sensor_left()
{
    return usDistances[SENSOR_LEFT];
}

double sensor_left_all()
{
    return min(usDistances[SENSOR_LEFT], usDistances[SENSOR_FRONTL]);
}

double sensor_right()
{
    return usDistances[SENSOR_RIGHT];
}

double sensor_right_all()
{
    return min(usDistances[SENSOR_RIGHT], usDistances[SENSOR_FRONTR]);
}

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
    int lastMove = STRAIGHT;

    for (;;)
    {
        // check if you can go straight
        if (sensor_front() > US_NEAR_TRIGGER && sensor_front_out() > US_MIN_TRIGGER && lastMove != BACKWARDS)
        {
            lastMove = STRAIGHT;
            stepperStartStraight(STEPPER_MAX_RPM);
        }
        // check if you can go left instead
        else if (sensor_left_all() > US_NEAR_TRIGGER && lastMove != RIGHT)
        {
            Serial.println("left");
            lastMove = LEFT;
            stepperStartTurnLeft(STEPPER_TURN_RPM);
            while (returnSteps() < STEPS_90)
            {
                if (sensor_left_all() < US_MIN_TRIGGER)
                {
                    break;
                }
                vTaskDelay(0);
            }
        }
        // check if you can go right instead
        else if (sensor_right_all() > US_NEAR_TRIGGER && lastMove != LEFT)
        {
            Serial.println("right");
            lastMove = RIGHT;
            stepperStartTurnRight(STEPPER_TURN_RPM);
            while (returnSteps() < STEPS_90)
            {
                if (sensor_right_all() < US_MIN_TRIGGER)
                {
                    break;
                }
                vTaskDelay(0);
            }
        }
        // must back off
        else
        {
            Serial.println("back");
            lastMove = BACKWARDS;
            stepperStartBackwards(STEPPER_MAX_RPM);
            while (returnSteps() < STEPER_STEPS_PER_ROT * 2)
            {
                vTaskDelay(0);
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
        if (sensor_front() < US_MIN_TRIGGER)
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
    const unsigned MIN_DRIVE_TIME = 2000;
    const unsigned MIN_TURN_TIME = 2000;
    static unsigned long driveTimer = millis();
    static unsigned long turnTimer = millis();

    if (detectTagCenter == 0 || udpConnection == false ||
        telnetConnection == false || missionMode == missions::NO_MISSION)
    {
        DEBUG_MSG("stop search");
        stepperStop();
        return;
    }
    // DEBUG_SER_VAR(detectTagCenter);

    if (innerLock) // inner lock
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
                if (sensor_front() > US_NEAR_TRIGGER)
                {
                    DEBUG_MSG("drive straight");
                    followMode = movement_state::STRAIGHT;
                    stepperStartStraight(STEPPER_MAX_RPM);
                }
                else
                {
                    while (1)
                    {
                        vTaskDelay(0);
                    }
                }
            }
            else // close to tag
            {
                DEBUG_MSG("Beacon reached");
                stepperStop();
                while (1)
                {
                    vTaskDelay(0);
                }
            }
        }
    }
    else // outer lock
    {
        if (detectTagCenter < TAG_CENTER - TAG_CENTER_DEADZONE_SMALL)
        {
            if (usDistances[SENSOR_RIGHT] > US_MIN_TRIGGER)
            {
                if ((millis() > driveTimer && millis() > turnTimer &&
                     followMode != movement_state::RIGHT) ||
                    !stepperIsRunning())
                {
                    DEBUG_MSG(" turn right");
                    followMode = movement_state::RIGHT;
                    stepperStartTurnRight(STEPPER_SLOW_TURN_RPM);
                }
            }
            else if (sensor_front() > US_NEAR_TRIGGER)
            {
                if (followMode != movement_state::STRAIGHT || !stepperIsRunning())
                {
                    DEBUG_MSG("cant turn right -> drive straight");
                    driveTimer = millis() + MIN_DRIVE_TIME;
                    followMode = movement_state::STRAIGHT;
                    stepperStartStraight(STEPPER_MAX_RPM);
                }
            }
            else if (usDistances[SENSOR_LEFT] > US_NEAR_TRIGGER)
            {
                if (followMode != movement_state::LEFT || !stepperIsRunning())
                {
                    DEBUG_MSG("cant turn right or go straight -> turn left");
                    turnTimer = millis() + MIN_TURN_TIME;
                    followMode = movement_state::LEFT;
                    stepperStartTurnLeft(STEPPER_TURN_RPM);
                }
            }
            else
            {
                if (followMode != movement_state::BACKWARDS || !stepperIsRunning())
                {
                    DEBUG_MSG("cant turn  go straight -> go backwards");
                    turnTimer = millis() + MIN_TURN_TIME;
                    followMode = movement_state::BACKWARDS;
                    stepperStartBackwards(STEPPER_MAX_RPM);
                }
            }
        }
        else if (detectTagCenter > TAG_CENTER + TAG_CENTER_DEADZONE_SMALL)
        {
            if (usDistances[SENSOR_LEFT] > US_MIN_TRIGGER)
            {
                if ((millis() > driveTimer && millis() > turnTimer &&
                     followMode != movement_state::LEFT) ||
                    !stepperIsRunning())
                {
                    DEBUG_MSG(" turn left");
                    followMode = movement_state::LEFT;
                    stepperStartTurnLeft(STEPPER_SLOW_TURN_RPM);
                }
            }
            else if (sensor_front() > US_NEAR_TRIGGER)
            {
                if (followMode != movement_state::STRAIGHT || !stepperIsRunning())
                {
                    DEBUG_MSG("cant turn left -> drive straight");
                    driveTimer = millis() + MIN_DRIVE_TIME;
                    followMode = movement_state::STRAIGHT;
                    stepperStartStraight(STEPPER_MAX_RPM);
                }
            }
            else if (usDistances[SENSOR_RIGHT] > US_NEAR_TRIGGER)
            {
                if (followMode != movement_state::RIGHT || !stepperIsRunning())
                {
                    DEBUG_MSG("cant turn left or go straight -> turn right");
                    turnTimer = millis() + MIN_TURN_TIME;
                    followMode = movement_state::RIGHT;
                    stepperStartTurnRight(STEPPER_TURN_RPM);
                }
            }
            else
            {
                if (followMode != movement_state::BACKWARDS || !stepperIsRunning())
                {
                    DEBUG_MSG("cant turn  go straight -> go backwards");
                    turnTimer = millis() + MIN_TURN_TIME;
                    followMode = movement_state::BACKWARDS;
                    stepperStartBackwards(STEPPER_MAX_RPM);
                }
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
    
    for (;;)
    {
        // wait for ultrasonic
        while (ultrasonicStarted == false){
            DEBUG_MSG("waiting for us sensors...");
            delay(1000);
        }
        delay(1000);
        reposition();

        // if (udpConnection == false)
        // {
        //     tagLock = false;
        //     stepperStop();
        //     waitForUDP();
        // }
        // else if (telnetConnection == false)
        // {
        //     tagLock = false;
        //     stepperStop();
        //     waitForTelnet();
        // }
        // else if (missionMode == missions::NO_MISSION)
        // {
        //     stepperStop();
        //     askForMission();
        // }
        // else if (tagLock == false)
        // {
        //     stepperStop();
        //     searchForTag();
        //     tagTimeoutTimer = millis();
        // }
        // else if (detectTagCenter > 0)
        // {
        //     tagTimeoutTimer = millis();
        //     followTag();
        // }
        // else if (millis() - tagTimeoutTimer > 1000)
        // {
        //     DEBUG_MSG("tagTimeout: tag lock lost");
        //     tagLock = false;
        // }
        vTaskDelay(0);
    }
    Serial.println("carControlTask closed");
    vTaskDelete(NULL);
}