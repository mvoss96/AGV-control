#include <Arduino.h>
#include "defines.hpp"
#include "car_control.hpp"
#include "stepper_motor.hpp"
#include "april_tag.hpp"
#include "telnet_debug.hpp"
#include "ultrasonic.hpp"
#include "wifi.hpp"
#include "object_recognition.hpp"

extern unsigned volatile detectTagCenter;
extern double detectTagSize;
extern bool udpConnection;
extern bool telnetConnection;
extern uint8_t missionMode;
extern bool ultrasonicEnable;
extern bool ultrasonicStarted;
extern double usDistances[NUM_SENSORS];

extern unsigned robotStatus;
extern unsigned robotCargo;
extern unsigned robotRequest;

namespace
{
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
    bool tagLock = false;
    bool innerCircle = false;

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

    auto doNothingUntilStop = []()
    {
        for (;;)
        {
            if (stopMode())
            {
                break;
            }
            vTaskDelay(0);
        }
    };

    auto goLeftSteps = [](unsigned steps)
    {
        stepperStartTurnLeft(STEPPER_TURN_RPM);
        while (returnSteps() < steps)
        {
            stepperStartTurnLeft(STEPPER_TURN_RPM);
            vTaskDelay(0);
        }
    };

    auto goRightSteps = [](unsigned steps)
    {
        stepperStartTurnRight(STEPPER_TURN_RPM);
        while (returnSteps() < steps)
        {
            stepperStartTurnRight(STEPPER_TURN_RPM);
            vTaskDelay(0);
        }
    };

    auto goStraightSteps = [](unsigned steps)
    {
        stepperStartStraight(STEPPER_MAX_RPM);
        while (returnSteps() < steps)
        {
            stepperStartStraight(STEPPER_MAX_RPM);
            vTaskDelay(0);
        }
    };

    auto goBackSteps = [](unsigned steps)
    {
        stepperStartBackwards(STEPPER_MAX_RPM);
        while (returnSteps() < steps)
        {
            stepperStartBackwards(STEPPER_MAX_RPM);
            vTaskDelay(0);
        }
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
        robotStatus = ROBOT_IDLE;
        robotCargo = CARGO_EMPTY;
        robotRequest = REQUEST_NO_REQUEST;

        DEBUG_MSG("ask for mission");
        telnet.println("please choose a mission:");
        telnet.println("d-> deliver");
        telnet.println("gg -> get gummy bear");
        telnet.println("gc -> get cotton wool");
        telnet.println("gb -> get ping pong ball");
        while (missionMode == missions::NO_MISSION)
            vTaskDelay(0);

        robotStatus = ROBOT_APPROACHING_STATION;
        if (missionMode == missions::DELIVER)
        {
            bool l;
            for (int i = 0; i < 5; i++)
            {
                l = objectLoaded();
                if (l == true)
                    break;
                DEBUG_MSG("please load an object...");
                delay(2000);
            }
            unsigned c = measureObject();

            switch (c)
            {
            case RECOGNITION_BALL:
                robotCargo = CARGO_BALL;
                DEBUG_MSG("BALL loaded");
                break;
            case RECOGNITION_GUMMY:
                robotCargo = CARGO_GUMMY;
                DEBUG_MSG("GUMMY loaded");
                break;
            case RECOGNITION_COTTON:
                robotCargo = CARGO_COTTON;
                DEBUG_MSG("COTTON loaded");
                break;
            case RECOGNITION_ERROR:
                DEBUG_MSG("object reconition error");
                robotCargo = CARGO_EMPTY;
                break;
            }
            delay(2000);
        }
        else
        {
            for (int i = 0; i < 5; i++)
            {
                if (objectLoaded() == false)
                    break;
                DEBUG_MSG("make sure container is empty...");
                delay(2000);
            }
            if (missionMode == missions::GET_BALL)
            {
                robotCargo = CARGO_EMPTY;
                robotRequest = REQUEST_LOAD_BALL;
            }
            else if (missionMode == missions::GET_GUMMY)
            {
                robotCargo = CARGO_EMPTY;
                robotRequest = REQUEST_LOAD_GUMMY;
            }
            else if (missionMode == missions::GET_COTTON)
            {
                robotCargo = CARGO_EMPTY;
                robotRequest = REQUEST_LOAD_COTTON;
            }
            delay(2000);
        }
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
                sensor_left() > 2 && sensor_right() > 2 && lastMove != BACKWARDS)
            {
                if (detectTagCenter != 0)
                {
                    DEBUG_MSG("reposition: tag in view");
                    DEBUG_VAR(detectTagSize);
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
                    DEBUG_VAR(detectTagSize);
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
                stepperStartTurnRight(STEPPER_TURN_RPM);
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
                ultrasonicPrint();
                lastMove = BACKWARDS;
                stepperStop();
                stepperStartBackwards(STEPPER_MAX_RPM);
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
                delay(500);
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
                DEBUG_VAR(detectTagSize);
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
                searchStartTime = millis();
                numChanges = 0;
            }
            else if (sensor_front_all() < US_MIN_TRIGGER)
            {
                DEBUG_MSG("search: back");
                stepperStop();
                stepperStartBackwards(STEPPER_MAX_RPM);
                while (returnSteps() < STEPER_STEPS_PER_ROT * 0.5)
                {
                    stepperStartBackwards(STEPPER_MAX_RPM);
                    if (stopMode())
                    {
                        DEBUG_MSG("search: back stop");
                        break;
                    }
                    vTaskDelay(10);
                }
            }
            else if (sensor_left_all() < US_MIN_TRIGGER && sensor_right_all() < US_MIN_TRIGGER)
            {
                DEBUG_MSG("search: cant turn");
                stepperStop();
                reposition();
                searchStartTime = millis();
            }
            else
            {
                if (dir && sensor_left_all() > US_MIN_TRIGGER)
                {
                    DEBUG_MSG("search: turn left");
                    stepperStop();
                    stepperStartTurnLeft(STEPPER_TURN_RPM);
                    while (returnSteps() < STEPS_360)
                    {
                        stepperStartTurnLeft(STEPPER_TURN_RPM);
                        if (sensor_left_all() < US_MIN_TRIGGER || detectTagCenter != 0 || stopMode())
                        {
                            DEBUG_MSG("search: turn left stop");
                            break;
                        }
                        vTaskDelay(10);
                    }
                }
                else if (!dir && sensor_right_all() > US_MIN_TRIGGER)
                {
                    DEBUG_MSG("search: turn right");
                    stepperStop();
                    stepperStartTurnRight(STEPPER_TURN_RPM);
                    while (returnSteps() < STEPS_360)
                    {
                        stepperStartTurnRight(STEPPER_TURN_RPM);
                        if (sensor_right_all() < US_MIN_TRIGGER || detectTagCenter != 0 || stopMode())
                        {
                            DEBUG_MSG("search: turn right stop");
                            break;
                        }
                        vTaskDelay(10);
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
            vTaskDelay(10);
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
        auto avoidObstacle = []()
        {
            int lturns = 0;
            int intendedMove = STRAIGHT;
            int lastObMove = STRAIGHT;
            while (1)
            {
                DEBUG_VAR(lturns);
                if (sensor_front_all() > US_NEAR_TRIGGER &&
                    sensor_front_out() > US_MIN_TRIGGER)
                {
                    DEBUG_MSG("obstacle: front is free");
                    if (lturns == 0)
                    {
                        DEBUG_MSG("obstacle: turned enough");
                        break;
                    }
                    else
                    {
                        DEBUG_MSG("obstacle: straight");
                        stepperStartStraight(STEPPER_MAX_RPM);
                        while (returnSteps() < STEPER_STEPS_PER_ROT * 2.0)
                        {
                            if (sensor_front() < US_MIN_TRIGGER || sensor_front_out() < 2)
                            {
                                DEBUG_MSG("obstacle: stopped straight");
                                lastObMove = STRAIGHT;
                                goBackSteps(STEPER_STEPS_PER_ROT * 0.1);
                                stepperStop();
                                break;
                            }
                            vTaskDelay(0);
                        }

                        while (sensor_right_all() > US_MIN_TRIGGER && lturns > 0)
                        {
                            DEBUG_MSG("obstacle: right back");
                            lturns -= 1;
                            goRightSteps(STEPS_90 * 0.5);
                        }
                        while (sensor_left_all() > US_MIN_TRIGGER && lturns < 0)
                        {
                            DEBUG_MSG("obstacle: left back");
                            lturns += 1;
                            goLeftSteps(STEPS_90 * 0.5);
                        }
                    }
                }
                else if (sensor_left_all() > US_MIN_TRIGGER && lastObMove != RIGHT)
                {
                    DEBUG_MSG("obstacle: left");
                    lastObMove = LEFT;
                    lturns += 1;
                    goLeftSteps(STEPS_90 * 0.5);
                }
                else if (sensor_right_all() > US_MIN_TRIGGER)
                {
                    DEBUG_MSG("obstacle: right");
                    lastObMove = RIGHT;
                    lturns -= 1;
                    goRightSteps(STEPS_90 * 0.5);
                }
                else
                {
                    DEBUG_MSG("obstacle: back");
                    lastObMove = BACKWARDS;
                    goBackSteps(STEPER_STEPS_PER_ROT);
                }
            }
        };

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
                DEBUG_VAR(detectTagSize);
                stepperStop();
                lockedOn = true;
            }

            // check if we are in inner circle
            if (!innerCircle && detectTagSize > TAG_CLOSE_SIZE)
            {
                DEBUG_MSG("follow: we are inside inner circle");
                DEBUG_VAR(detectTagSize);
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
                    DEBUG_MSG("follow: turn right");
                    lastMove = RIGHT;
                    stepperStartTurnRight(STEPPER_SLOW_TURN_RPM);
                    while (sensor_right_all() > US_MIN_TRIGGER)
                    {
                        stepperStartTurnRight(STEPPER_SLOW_TURN_RPM);
                        if (stopMode() || innerLock())
                        {
                            DEBUG_MSG("follow: stopped turning right");
                            break;
                        }
                        vTaskDelay(10);
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
                    DEBUG_MSG("follow: turn left");
                    lastMove = LEFT;
                    stepperStartTurnLeft(STEPPER_SLOW_TURN_RPM);
                    while (sensor_left_all() > US_MIN_TRIGGER)
                    {
                        stepperStartTurnLeft(STEPPER_SLOW_TURN_RPM);
                        if (stopMode() || innerLock())
                        {
                            DEBUG_MSG("follow: stopped turning left");
                            break;
                        }
                        vTaskDelay(10);
                    }
                }
                else
                {
                    DEBUG_MSG("follow: cant turn left -> intend Straight");
                }
            }

            // check if we are near station
            else if (innerCircle && sensor_front() < 25)
            {
                DEBUG_MSG("follow: near station!");
                stepperStop();
                stepperStartStraight(15);
                while (1)
                {
                    if (sensor_front_all() < US_BASE_TRIGGER)
                    {
                        break;
                    }
                    vTaskDelay(0);
                }
                DEBUG_MSG("follow: stopped because we reached station");
                goStraightSteps(STEPER_STEPS_PER_ROT * 0.15);
                stepperStop();
                robotStatus = ROBOT_STOPPED_NEAR_STATION;
                missionMode == missions::WAITING;
                while (missionMode != missions::DRIVING_AWAY)
                {
                    vTaskDelay(10);
                }
                DEBUG_MSG("DriveBack: started ");
                if (objectLoaded())
                {
                    int cargo = measureObject();
                    if (cargo == CARGO_BALL)
                    {
                        DEBUG_MSG("new cargo: BALL");
                    }
                    else if (cargo == CARGO_GUMMY)
                    {
                        DEBUG_MSG("new cargo: GUMMY");
                    }
                    else if (cargo == CARGO_COTTON)
                    {
                        DEBUG_MSG("new cargo: COTTON");
                    }
                    else
                    {
                        DEBUG_MSG("new cargo: UNKNOWN");
                    }
                }
                else
                {
                    DEBUG_MSG("new cargo: NONE");
                }
                goBackSteps(STEPER_STEPS_PER_ROT * 0.5);
                DEBUG_MSG("DriveBack: turn around");
                goLeftSteps(STEPS_90 * 2);
                long startTime = millis();
                while (true)
                {
                    if (millis() - startTime > DRIVE_BACK_TIMEOUT)
                    {
                        DEBUG_MSG("DriveBack: timeout");
                        break;
                    }
                    if (sensor_front_all() < US_MIN_TRIGGER)
                    {
                        DEBUG_MSG("DriveBack: obstacle");
                        break;
                    }
                }
                DEBUG_MSG("DriveBack: turn around");
                goLeftSteps(STEPS_90 * 2);
                DEBUG_MSG("DriveBack: finished");
                stepperStop();
                innerCircle = false;
                tagLock = false;
                missionMode = NO_MISSION;
                return;
            }

            // go straight if possible
            else if (intendedMove == STRAIGHT && sensor_front() > US_NEAR_TRIGGER &&
                     sensor_front_out() > 5)
            {
                while (sensor_left() < 2 && sensor_right() < 2)
                {
                    DEBUG_MSG("follow: make space -> back");
                    stepperStartBackwards(STEPPER_MAX_RPM);
                    vTaskDelay(10);
                    stepperStop();
                }
                while (sensor_left() < 2)
                {
                    DEBUG_MSG("follow: make space -> right");
                    stepperStartTurnRight(STEPPER_TURN_RPM);
                    vTaskDelay(10);
                    stepperStop();
                }
                while (sensor_right() < 2)
                {
                    DEBUG_MSG("follow: make space -> left");
                    stepperStartTurnLeft(STEPPER_TURN_RPM);
                    vTaskDelay(10);
                    stepperStop();
                }

                if (lastMove != STRAIGHT)
                {
                    stepperStop();
                    DEBUG_MSG("follow: go straight");
                }
                lastMove = STRAIGHT;
                stepperStartStraight(STEPPER_MAX_RPM);
                vTaskDelay(10);
            }

            // cant go straight
            else
            {
                DEBUG_MSG("follow: cant go straight -> drive around obstacle");
                stepperStop();
                avoidObstacle();
            }
            vTaskDelay(10);
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
}

void controlCarTask(void *argument)
{
    Serial.print("carControlTask is running on: ");
    Serial.println(xPortGetCoreID());
    unsigned long tagTimeoutTimer = 0;
    robotStatus = ROBOT_IDLE;
    robotCargo = CARGO_EMPTY;
    robotRequest = REQUEST_NO_REQUEST;

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
        if (detectTagCenter == 0)
        {
            tagLock = false;
            stepperStop();
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