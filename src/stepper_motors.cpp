#include <Arduino.h>
#include "defines.hpp"
#include "stepper_motor.hpp"
#include "BasicStepperDriver.h"
#include "SyncDriver.h"

namespace
{
    BasicStepperDriver stepper(STEPER_STEPS_PER_ROT, STEPPER_L_DIR, STEPPER_L_STEP, STEPPER_L_SLEEP);
    BasicStepperDriver stepper2(STEPER_STEPS_PER_ROT, STEPPER_R_DIR, STEPPER_R_STEP, STEPPER_R_SLEEP);
    SyncDriver controller(stepper, stepper2);
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
    stepper.setSpeedProfile(stepper.LINEAR_SPEED, 1000, 1000);
    stepper2.setSpeedProfile(stepper.LINEAR_SPEED, 1000, 1000);
    controller.begin(STEPPER_MAX_RPM, 1);
    stepperStop();
}

void stepperStartTurnLeft(unsigned int t)
{
    controller.enable();
    controller.startMove(STEPER_STEPS_PER_ROT * 2, STEPER_STEPS_PER_ROT * 2, t);
}

void stepperStartTurnRight(unsigned int t)
{
    controller.enable();
    controller.startMove(-STEPER_STEPS_PER_ROT * 2, -STEPER_STEPS_PER_ROT * 2, t);
}

void stepperStartStraight()
{
    controller.enable();
    controller.startMove(STEPER_STEPS_PER_ROT * 100, -STEPER_STEPS_PER_ROT * 100);
}

void stepperStartBackwards()
{
    controller.enable();
    controller.startMove(-STEPER_STEPS_PER_ROT * 100, STEPER_STEPS_PER_ROT * 100);
}

void stepperStop()
{
    controller.stop();
    controller.disable();
}

bool stepperIsRunning()
{
    return controller.isRunning();
}