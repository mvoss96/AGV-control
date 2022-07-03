#include <Arduino.h>
#include "defines.hpp"
#include "car_control.hpp"
#include "stepper_motor.hpp"
#include "april_tag.hpp"
#include "telnet_debug.hpp"
#include "ultrasonic.hpp"

extern unsigned volatile detectTagCenter;
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
    bool innerCircle = false;
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
    DEBUG_MSG("reposition: start");
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
            while (returnSteps() < STEPS_90 / 2)
            {
                stepperStartTurnLeft(STEPPER_TURN_RPM);
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
            while (returnSteps() < STEPS_90)
            {
                stepperStartTurnRight(STEPPER_TURN_RPM);
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
            while (returnSteps() < STEPER_STEPS_PER_ROT * 0.5)
            {
                stepperStartBackwards(STEPPER_MAX_RPM);
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
    DEBUG_MSG("search: start");
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
            while (returnSteps() < STEPER_STEPS_PER_ROT * 0.5)
            {
                stepperStartBackwards(STEPPER_MAX_RPM);
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
                while (returnSteps() < STEPS_360)
                {
                    stepperStartTurnLeft(STEPPER_TURN_RPM);
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
                while (returnSteps() < STEPS_360)
                {
                    stepperStartTurnRight(STEPPER_TURN_RPM);
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

bool innerLock()
{
    unsigned localTagCenter = detectTagCenter;
    return (localTagCenter > TAG_CENTER - TAG_CENTER_DEADZONE_SMALL &&
            localTagCenter < TAG_CENTER + TAG_CENTER_DEADZONE_SMALL);
}

void followTag()
{
    DEBUG_MSG("follow: start");
    int lastMove = NONE;
    bool lockedOn = false;
    int intendedMove = STRAIGHT;
    unsigned long taglastSeen = 0;

    for (;;)
    {
        // check if we lost tag or connection
        if (detectTagCenter == 0)
        {

            DEBUG_MSG("follow: stopped because tag is no longer in view");
            tagLock = false;
            stepperStop();
            return;
        }

        if (stopMode())
        {
            DEBUG_MSG("follow: stopped");
            stepperStop();
            return;
        }

        // check if we left inner lock
        unsigned temp = detectTagCenter;
        if (lockedOn && (temp < TAG_CENTER - TAG_CENTER_DEADZONE ||
                         temp > TAG_CENTER + TAG_CENTER_DEADZONE))
        {
            DEBUG_MSG("follow: lockedOn lost");
            DEBUG_VAR(temp);
            stepperStop();
            lockedOn = false;
        }

        // check if we entered inner lock
        else if (!lockedOn && innerLock())
        {
            DEBUG_MSG("follow: lockedOn");
            DEBUG_VAR(detectTagCenter);
            stepperStop();
            lockedOn = true;
        }

        // check if we are in inner circle
        if (!innerCircle && detectTagSize > TAG_CLOSE_SIZE)
        {
            DEBUG_MSG("follow: we are inside inner circle");
            innerCircle = true;
        }

        // what move to we want to do?
        intendedMove = STRAIGHT;
        if (!lockedOn && detectTagCenter < TAG_CENTER)
        {
            if (lastMove != RIGHT)
            {
                DEBUG_MSG("follow: intend right");
            }
            intendedMove = (sensor_right_all() < US_MIN_TRIGGER) ? STRAIGHT : RIGHT;
            if (intendedMove == RIGHT)
            {
                lastMove = RIGHT;
                while (sensor_right_all() > US_MIN_TRIGGER)
                {
                    stepperStartTurnRight(STEPPER_SLOW_TURN_RPM);
                    if (stopMode() || innerLock())
                    {
                        DEBUG_MSG("follow: stopped turning right");
                        break;
                    }
                    vTaskDelay(0);
                }
            }
            else
            {
                DEBUG_MSG("follow: cant turn right -> intend straight");
            }
        }
        else if (!lockedOn && detectTagCenter > TAG_CENTER)
        {
            if (lastMove != LEFT)
            {
                DEBUG_MSG("follow: intend left");
            }
            intendedMove = (sensor_left_all() < US_MIN_TRIGGER) ? STRAIGHT : LEFT;
            if (intendedMove == LEFT)
            {
                lastMove = LEFT;
                while (sensor_left_all() > US_MIN_TRIGGER)
                {
                    stepperStartTurnLeft(STEPPER_SLOW_TURN_RPM);
                    if (stopMode() || innerLock())
                    {
                        DEBUG_MSG("follow: stopped turning left");
                        break;
                    }
                    vTaskDelay(0);
                }
            }
            else
            {
                DEBUG_MSG("follow: cant turn left -> intend Straight");
            }
        }

        // go straight if possible
        else if (intendedMove == STRAIGHT && sensor_front() > US_NEAR_TRIGGER &&
                 sensor_front_out() > US_MIN_TRIGGER && sensor_left() > US_MIN_TRIGGER &&
                 sensor_right() > US_MIN_TRIGGER)
        {
            if (lastMove != STRAIGHT)
            {
                DEBUG_MSG("follow: go straight");
            }
            lastMove = STRAIGHT;
            stepperStartStraight(STEPPER_MAX_RPM);
        }

        // check if we are near station
        else if (innerCircle && sensor_front() < US_NEAR_TRIGGER)
        {
            DEBUG_MSG("follow: near station!");
            while (detectTagCenter < TAG_CENTER - TAG_CENTER_DEADZONE_SMALL)
            {
                stepperStartTurnRight(STEPPER_SLOW_TURN_RPM);
                vTaskDelay(0);
            }
            while (detectTagCenter > TAG_CENTER + TAG_CENTER_DEADZONE_SMALL)
            {
                stepperStartTurnLeft(STEPPER_SLOW_TURN_RPM);
                vTaskDelay(0);
            }
            while (1)
            {
                stepperStartStraight(STEPPER_MAX_RPM);
                if (sensor_front_all() < US_BASE_TRIGGER)
                {
                    break;
                }
            }
            DEBUG_MSG("follow: stopped because we reached station");
            stepperStop();
            for (;;)
            {
                delay(0);
            }
        }
        // cant go straight
        else
        {
            DEBUG_MSG("follow: cant go straight -> drive around obstacle");
            stepperStop();
            for (;;)
            {
                delay(0);
            }
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
    unsigned long tagTimeoutTimer = 0;

    // for (;;)
    // {
    //     // detectTagCenter = 633;
    //     // Serial.println("-----");
    //     // Serial.printf("inner: %i\n", innerLock());
    //     // Serial.printf("outer: %i\n", outerLock());
    //     stepperStartStraight(STEPPER_TURN_RPM);
    //     delay(0);
    // }

    for (;;)
    {
        if (detectTagCenter == 0)
        {
            tagLock = false;
            stepperStop();
        }
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