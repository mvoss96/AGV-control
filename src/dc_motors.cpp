#include <Arduino.h>
#include "dc_motors.hpp"
#include "defines.hpp"

static volatile long stepsL = 0, stepsR = 0;

void IRAM_ATTR ISR_countA()
{
    stepsL += 1;
}

void IRAM_ATTR ISR_countB()
{
    stepsR += 1;
}

DcMotor::DcMotor()
{
    Serial.println("motorSetup called");
    stepsL = 0;
    stepsR = 0;
    // set pin Output states
    pinMode(PIN_MOTOR_A1, OUTPUT);
    pinMode(PIN_MOTOR_A2, OUTPUT);
    pinMode(PIN_MOTOR_B1, OUTPUT);
    pinMode(PIN_MOTOR_B2, OUTPUT);
    pinMode(PIN_ENC_A, INPUT_PULLDOWN);
    pinMode(PIN_ENC_B, INPUT_PULLDOWN);

    // initialize PWM channels
    ledcSetup(MOTOR_PWM_A1, MOTOR_PWM_FRQ, MOTOR_PWM_RES);
    ledcSetup(MOTOR_PWM_A2, MOTOR_PWM_FRQ, MOTOR_PWM_RES);
    ledcSetup(MOTOR_PWM_B1, MOTOR_PWM_FRQ, MOTOR_PWM_RES);
    ledcSetup(MOTOR_PWM_B2, MOTOR_PWM_FRQ, MOTOR_PWM_RES);

    // attach channels to GPIO
    ledcAttachPin(PIN_MOTOR_A1, MOTOR_PWM_A1);
    ledcAttachPin(PIN_MOTOR_A2, MOTOR_PWM_A2);
    ledcAttachPin(PIN_MOTOR_B1, MOTOR_PWM_B1);
    ledcAttachPin(PIN_MOTOR_B2, MOTOR_PWM_B2);

    // make sure the motors are stopped
    stop();

    // attach Intterupts:
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), ISR_countA, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), ISR_countB, RISING);
}

void DcMotor::brake()
{
    ledcWrite(MOTOR_PWM_A1, 255);
    ledcWrite(MOTOR_PWM_A2, 255);
    ledcWrite(MOTOR_PWM_B1, 255);
    ledcWrite(MOTOR_PWM_B2, 255);
}

void DcMotor::stop()
{
    stopL();
    stopR();
}

void DcMotor::stopL()
{
    // Serial.println("stopped left");
    motorLrunning = false;
    ledcWrite(MOTOR_PWM_A1, 0);
    ledcWrite(MOTOR_PWM_A2, 0);
}

void DcMotor::stopR()
{
    // Serial.println("stopped right");
    motorRrunning = false;
    ledcWrite(MOTOR_PWM_B1, 0);
    ledcWrite(MOTOR_PWM_B2, 0);
}

void DcMotor::spinL(byte dc, bool dir)
{
    dirL = dir;
    motorLrunning = true;
    if (dir)
    {
        ledcWrite(MOTOR_PWM_A1, dc);
        ledcWrite(MOTOR_PWM_A2, 0);
    }
    else
    {
        ledcWrite(MOTOR_PWM_A1, 0);
        ledcWrite(MOTOR_PWM_A2, dc);
    }
}

void DcMotor::spinR(byte dc, bool dir)
{
    dirR = dir;
    motorRrunning = true;
    if (dir)
    {
        ledcWrite(MOTOR_PWM_B1, dc);
        ledcWrite(MOTOR_PWM_B2, 0);
    }
    else
    {
        ledcWrite(MOTOR_PWM_B1, 0);
        ledcWrite(MOTOR_PWM_B2, dc);
    }
}

void DcMotor::spinBoth(byte dc, bool dir)
{
    spinL(dc, dir);
    spinR(dc, dir);
}

void DcMotor::spinStepsL(long steps, bool dir)
{
    dirL = dir;
    motorLrunning = true;
    moveFinishedL = false;
    stepsL = 0;
    targetStepsL = steps;

    // dbg("targetStepsL", targetStepsL);
}

void DcMotor::spinStepsR(long steps, bool dir)
{
    dirR = dir;
    motorRrunning = true;
    moveFinishedR = false;
    stepsR = 0;
    targetStepsR = steps;

    // dbg("targetStepsR", targetStepsR);
}

void DcMotor::driveStraight(long dist, bool dir)
{
    int steps = 1;
    for (int i = 0; i < dist / steps; i++)
    {
        spinStepsL(steps, dir);
        spinStepsR(steps, dir);
        while (!moveFinishedL || !moveFinishedR)
        {
            yield();
        }
    }
    brake();
}

void DcMotor::driveTurn(int dist, bool dir)
{
    int steps = 1;
    for (int i = 0; i < dist / steps; i++)
    {
        spinStepsL(steps, dir);
        spinStepsR(steps, !dir);
        while (!moveFinishedL || !moveFinishedR)
        {
            yield();
        }
    }
    brake();
}

void DcMotor::printSteps()
{
    Serial.print(getStepsL());
    Serial.print(" ");
    Serial.println(getStepsR());
}

long DcMotor::getStepsL()
{
    return stepsL;
}

long DcMotor::getStepsR()
{
    return stepsR;
}

void DcMotor::control()
{
    int speed = 190;
    if (motorLrunning)
    {
        if (targetStepsL <= stepsL)
        {
            stopL();
            // printSteps();
            moveFinishedL = true;
        }
        else
        {
            spinL(speed, dirL);
        }
    }
    if (motorRrunning)
    {
        if (targetStepsR <= stepsR)
        {
            stopR();
            // printSteps();
            moveFinishedR = true;
        }
        else
        {
            spinR(speed, dirR);
        }
    }
}

bool DcMotor::getBusy()
{
    return busy;
}
