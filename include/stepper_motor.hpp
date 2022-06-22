#pragma once

void steppersControlTask(void *argument);

void stepperMotorsInit();

void stepperStartTurnLeft(unsigned int t = 0);

void stepperStartTurnRight(unsigned int t = 0);

void stepperStartStraight();

void stepperStartBackwards();

void stepperStop();

bool stepperIsRunning();