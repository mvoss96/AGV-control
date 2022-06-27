#include <Arduino.h>
#include "defines.hpp"
#include "stepper_motor.hpp"
#include "BasicStepperDriver.h"
#include "SyncDriver.h"

namespace
{
    BasicStepperDriver stepper(STEPER_STEPS_PER_ROT, STEPPER_L_DIR, STEPPER_L_STEP, STEPPER_L_SLEEP);
    BasicStepperDriver stepper2(STEPER_STEPS_PER_ROT, STEPPER_R_DIR, STEPPER_R_STEP, STEPPER_R_SLEEP);
    //SyncDriver controller(stepper, stepper2);
    MultiDriver controller(stepper, stepper2);
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
    controller.enable();
    controller.setRPM(rpm == 0 ? STEPPER_MAX_RPM : rpm);
    controller.startMove(STEPER_STEPS_PER_ROT * WHEEL_ROTS_360, STEPER_STEPS_PER_ROT * WHEEL_ROTS_360);
}

void stepperStartTurnRight(unsigned int rpm)
{
    controller.enable();
    controller.setRPM(rpm == 0 ? STEPPER_MAX_RPM : rpm);
    controller.startMove(-STEPER_STEPS_PER_ROT * WHEEL_ROTS_360, -STEPER_STEPS_PER_ROT * WHEEL_ROTS_360);
}

void stepperStartStraight(unsigned int rpm)
{
    controller.enable();
    controller.setRPM(rpm == 0 ? STEPPER_MAX_RPM : rpm);
    controller.startMove(STEPER_STEPS_PER_ROT * 100, -STEPER_STEPS_PER_ROT * 100);
}

void stepperStartStraightC(unsigned int rpm1,unsigned int rpm2)
{
    controller.enable();
    stepper.setRPM(rpm1);
    stepper2.setRPM(rpm2);
    controller.startMove(STEPER_STEPS_PER_ROT * 100, -STEPER_STEPS_PER_ROT * 100);
}

void stepperStartBackwards(unsigned int rpm)
{
    controller.enable();
    controller.setRPM(rpm == 0 ? STEPPER_MAX_RPM : rpm);
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

void setRpmStepperL(int rpm)
{
    stepper.setRPM(rpm);
}

void setRpmStepperR(int rpm)
{
    stepper2.setRPM(rpm);
}