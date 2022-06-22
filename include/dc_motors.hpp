#pragma once

void IRAM_ATTR ISR_countA();
void IRAM_ATTR ISR_countB();
void controlMotorTask(void *argument);


class DcMotor
{
private:
    int speed = 200;
    bool motorLrunning = false, motorRrunning = false;
    long targetStepsL = 0, targetStepsR = 0;
    bool moveFinishedL = false, moveFinishedR = false, dirL = true, dirR = true, busy = false;

public:
    DcMotor();
    void setup();
    void brake();
    void stop();
    void stopL();
    void stopR();
    void spinL(byte dc, bool dir);
    void spinR(byte dc, bool dir);
    void spinBoth(byte dc, bool dir);
    void spinStepsL(long steps, bool dir);
    void spinStepsR(long steps, bool dir);
    void driveStraight(long dist, bool dir);
    void driveTurn(int dist, bool dir);
    void printSteps();
    long getStepsL();
    long getStepsR();
    void control();
    bool getBusy();
};
