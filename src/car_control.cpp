#include <Arduino.h>
#include "defines.hpp"
#include "car_control.hpp"
#include "stepper_motor.hpp"
#include "april_tag.hpp"
#include "telnet_debug.hpp"
#include "ultrasonic.hpp"

extern unsigned int detectTagCenter;
extern double detectTagSize;
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
    NONE,
    RIGHT,
    LEFT,
    STRAIGHT,
    STRAIGHT_RIGHT,
    STRAIGHT_LEFT,
    BACKWARDS
};

bool stopMode()
{
    if (missionMode == NO_MISSION)
    {
        DEBUG_MSG("stopMode because NO_MISSION");
        return true;
    }
    if (!telnetConnection)
    {
        DEBUG_MSG("stopMode because telnet");
        return true;
    }
    if (!udpConnection)
    {
        DEBUG_MSG("stopMode because udp");
        return true;
    }
    return false;
}

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
    int lastMove = NONE;
    int repositionMoved = 0;
    unsigned long reposTimer = 0;

    for (;;)
    {
        if (stopMode())
        {
            DEBUG_MSG("reposition: stopped");
            return;
        }

        // check if you can go straight
        if (sensor_front() > US_NEAR_TRIGGER && sensor_front_out() > US_MIN_TRIGGER &&
            sensor_left() > US_MIN_TRIGGER && sensor_right() > US_MIN_TRIGGER && lastMove != BACKWARDS)
        {
            if (detectTagCenter != 0)
            {
                DEBUG_MSG("reposition: tag in view");
                return;
            }
            if (repositionMoved > REPOSITION_MAX_STOPS)
            {
                DEBUG_MSG("reposition: ended");
                return;
            }
            if (lastMove != STRAIGHT)
            {
                DEBUG_MSG("reposition: straight");
                reposTimer = millis();
                repositionMoved++;
            }
            if (millis() - reposTimer > REPOSITION_MAX_TIME)
            {
                DEBUG_MSG("reposition time ended");
                return;
            }
            lastMove = STRAIGHT;
            stepperStartStraight(STEPPER_MAX_RPM);
        }
        // check if you can go left instead
        else if (sensor_left_all() > US_NEAR_TRIGGER && lastMove != RIGHT)
        {
            DEBUG_MSG("reposition: left");
            lastMove = LEFT;
            stepperStartTurnLeft(STEPPER_TURN_RPM);
            while (returnSteps() < STEPS_90 / 2)
            {
                if (detectTagCenter != 0)
                {
                    DEBUG_MSG("reposition: tag in view");
                    return;
                }
                if (sensor_left_all() < US_MIN_TRIGGER || stopMode())
                {
                    DEBUG_MSG("reposition: break left");
                    break;
                }
                vTaskDelay(0);
            }
        }
        // check if you can go right instead
        else if (sensor_right_all() > US_NEAR_TRIGGER && lastMove != LEFT)
        {
            DEBUG_MSG("reposition: right");
            lastMove = RIGHT;
            stepperStartTurnRight(STEPPER_TURN_RPM);
            while (returnSteps() < STEPS_90)
            {
                if (detectTagCenter != 0)
                {
                    DEBUG_MSG("reposition: tag in view");
                    return;
                }
                if (sensor_right_all() < US_MIN_TRIGGER || stopMode())
                {
                    DEBUG_MSG("reposition: break right");
                    break;
                }
                vTaskDelay(0);
            }
        }
        // must back off
        else
        {
            DEBUG_MSG("reposition: back");
            lastMove = BACKWARDS;
            stepperStartBackwards(STEPPER_MAX_RPM);
            while (returnSteps() < STEPER_STEPS_PER_ROT * 0.5)
            {
                if (detectTagCenter != 0)
                {
                    DEBUG_MSG("reposition: tag in view");
                    return;
                }
                if (stopMode())
                {
                    DEBUG_MSG("reposition: break back");
                    break;
                }
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
    unsigned numChanges = 0;
    int lastMove = NONE;

    for (;;)
    {
        if (stopMode())
        {
            DEBUG_MSG("search: stopped");
            return;
        }
        if (detectTagCenter != 0)
        {
            DEBUG_MSG("search: tag in view");
            tagLock = true;
            return;
        }
        else if (millis() - searchStartTime > TAG_SEARCH_TIMEOUT)
        {
            DEBUG_MSG("search: timeout");
            reposition();
            searchStartTime = millis();
        }
        else if (numChanges > 2)
        {
            DEBUG_MSG("search: to many changes");
            reposition();
            numChanges = 0;
        }
        else if (sensor_front_all() < US_MIN_TRIGGER)
        {
            DEBUG_MSG("search: back");
            stepperStartBackwards(STEPPER_MAX_RPM);
            while (returnSteps() < STEPER_STEPS_PER_ROT * 0.5)
            {
                if (stopMode())
                {
                    break;
                }
                vTaskDelay(0);
            }
        }
        else if (sensor_left_all() < US_MIN_TRIGGER && sensor_right_all() < US_MIN_TRIGGER)
        {
            DEBUG_MSG("search: cant turn");
            stepperStop();
            reposition();
        }
        else
        {
            if (dir && sensor_left_all() > US_MIN_TRIGGER)
            {
                DEBUG_MSG("search: turn left");
                stepperStartTurnLeft(STEPPER_TURN_RPM);
                while (returnSteps() < STEPS_360)
                {
                    if (!stepperIsRunning() || sensor_left_all() < US_MIN_TRIGGER || detectTagCenter != 0 || stopMode())
                    {
                        break;
                    }
                    vTaskDelay(0);
                }
            }
            else if (!dir && sensor_right_all() > US_MIN_TRIGGER)
            {
                DEBUG_MSG("search: turn right");
                stepperStartTurnRight(STEPPER_TURN_RPM);
                while (returnSteps() < STEPS_360)
                {
                    if (!stepperIsRunning() || sensor_right_all() < US_MIN_TRIGGER || detectTagCenter != 0 || stopMode())
                    {
                        break;
                    }
                    vTaskDelay(0);
                }
            }
            else
            {
                DEBUG_MSG("search: obstacle, changing direction");
                stepperStop();
                numChanges++;
                dir = !dir;
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

    if (detectTagCenter == 0 || stopMode())
    {
        DEBUG_MSG("stop follow");
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
            DEBUG_VAR(detectTagSize);
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
                    DEBUG_VAR(detectTagSize);
                    vTaskDelay(1000);
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
    while (udpConnection == false)
    {
        DEBUG_MSG("carControlTask is waiting for UDP stream");
        vTaskDelay(1000);
    }
}

void waitForTelnet()
{
    while (telnetConnection == false)
    {
        DEBUG_MSG("carControlTask is waiting for telnet");
        vTaskDelay(1000);
    }
}

void waitForUs()
{
    while (ultrasonicStarted == false)
    {
        DEBUG_MSG("waiting for us sensors...");
        vTaskDelay(1000);
    }
    delay(1000); // additional delay needed for ultrasonic sensors
}

void controlCarTask(void *argument)
{
    Serial.print("carControlTask is running on: ");
    Serial.println(xPortGetCoreID());

    for (;;)
    {
        if (telnetConnection == false)
        {
            tagLock = false;
            stepperStop();
            waitForTelnet();
        }
        if (ultrasonicStarted == false)
        {
            tagLock = false;
            stepperStop();
            waitForUs();
        }
        if (udpConnection == false)
        {
            tagLock = false;
            stepperStop();
            waitForUDP();
        }
        if (missionMode == missions::NO_MISSION)
        {
            tagLock = false;
            stepperStop();
            askForMission();
        }
        if (tagLock == false)
        {
            stepperStop();
            searchForTag();
            tagTimeoutTimer = millis();
        }
        else if (detectTagCenter > 0)
        {
            followTag();
            tagTimeoutTimer = millis();
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