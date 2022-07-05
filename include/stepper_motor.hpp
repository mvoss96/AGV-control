#pragma once

void steppersControlTask(void *argument);

void stepperMotorsInit();

void stepperStartTurnLeft(unsigned int rpm = 0);

void stepperStartTurnRight(unsigned int rpm = 0);

void stepperStartStraight(unsigned int rpm = 0);

void stepperStartBackwards(unsigned int rpm = 0);

void stepperStop();

bool stepperIsRunning();

void setRpmStepperL(int rpm);

void setRpmStepperR(int rpm);

void stepperUpdate();

unsigned long returnSteps();