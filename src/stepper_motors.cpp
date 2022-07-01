#include <Arduino.h>
#include "defines.hpp"
#include "stepper_motor.hpp"
#include "BasicStepperDriver.h"
#include "SyncDriver.h"

namespace
{
    BasicStepperDriver stepper(STEPER_STEPS_PER_ROT, PIN_STEPPER_L_DIR, PIN_STEPPER_L_STEP, PIN_STEPPER_L_SLEEP);
    BasicStepperDriver stepper2(STEPER_STEPS_PER_ROT, PIN_STEPPER_R_DIR, PIN_STEPPER_R_STEP, PIN_STEPPER_R_SLEEP);
    // SyncDriver controller(stepper, stepper2);
    MultiDriver controller(stepper, stepper2);
    enum movement_state
    {
        STOPPED,
        RIGHT,
        LEFT,
        STRAIGHT,
        BACKWARDS
    };
    unsigned state;
}

void steppersControlTask(void *argument)
{
    Serial.print("steppersControlTask is running on: ");
    Serial.println(xPortGetCoreID());

    for (;;)
    {
        if (stepperIsRunning() == false)
        {
            controller.disable();
        }
        controller.nextAction();
        vTaskDelay(0);
    }
    Serial.println("steppersControlTask closed");
    vTaskDelete(NULL);
}

void stepperMotorsInit()
{
    Serial.println("initialize Stepper Motors");
    stepper.begin(STEPPER_MAX_RPM, 1);
    stepper2.begin(STEPPER_MAX_RPM, 1);
    stepper.setSpeedProfile(stepper.CONSTANT_SPEED, 5000, 5000);
    stepper2.setSpeedProfile(stepper.CONSTANT_SPEED, 5000, 5000);
    controller.begin(STEPPER_MAX_RPM, 1);
    stepperStop();
}

void stepperStartTurnLeft(unsigned int rpm)
{
    if (state != LEFT)
    {
        state = LEFT;
        controller.enable();
        controller.setRPM(rpm == 0 ? STEPPER_MAX_RPM : rpm);
        controller.startMove(STEPER_STEPS_PER_ROT * WHEEL_ROTS_360, STEPER_STEPS_PER_ROT * WHEEL_ROTS_360);
    }
}

void stepperStartTurnRight(unsigned int rpm)
{
    if (state != RIGHT)
    {
        state = RIGHT;
        controller.enable();
        controller.setRPM(rpm == 0 ? STEPPER_MAX_RPM : rpm);
        controller.startMove(-STEPER_STEPS_PER_ROT * WHEEL_ROTS_360, -STEPER_STEPS_PER_ROT * WHEEL_ROTS_360);
    }
}

void stepperStartStraight(unsigned int rpm)
{
    if (state != STRAIGHT)
    {
        state = STRAIGHT;
        controller.enable();
        controller.setRPM(rpm == 0 ? STEPPER_MAX_RPM : rpm);
        controller.startMove(STEPER_STEPS_PER_ROT * 100, -STEPER_STEPS_PER_ROT * 100);
    }
}

void stepperStartBackwards(unsigned int rpm)
{
    if (state != BACKWARDS)
    {
        state = BACKWARDS;
        controller.enable();
        controller.setRPM(rpm == 0 ? STEPPER_MAX_RPM : rpm);
        controller.startMove(-STEPER_STEPS_PER_ROT * 100, STEPER_STEPS_PER_ROT * 100);
    }
}

void stepperStop()
{
    state = STOPPED;
    controller.stop();
    controller.disable();
}

bool stepperIsRunning()
{
    return controller.isRunning();
}

void setRpmStepperL(int rpm)
{
    stepper.setRPM(rpm);
}

void setRpmStepperR(int rpm)
{
    stepper2.setRPM(rpm);
}

unsigned long returnSteps()
{
    return stepper.getStepsCompleted();
}