#include <Arduino.h>
#include "defines.hpp"
#include "ultrasonic.hpp"

//stores measured distances form ultrasonic sensors for use by other tasks
double usDistances[3] = {0};

/**
 * @brief private area
 * 
 */
namespace
{
    volatile unsigned long timerPulseStart[3] = {0};
    volatile unsigned long timerPulseDuration[3] = {0};
    volatile bool timerPulseFinished[3] = {0};
    const unsigned triggerPins[3] = {PIN_US0_TRIGGER, PIN_US1_TRIGGER, PIN_US2_TRIGGER};
    const unsigned echoPins[3] = {PIN_US0_ECHO, PIN_US1_ECHO, PIN_US2_ECHO};

    /**
     * @brief interrupt handler to time the upper flank of the echo signal
     *
     * @param n the indix of the sensor
     */
    void IRAM_ATTR pulseEcho(unsigned n)
    {
        if (digitalRead(echoPins[n]) == HIGH)
        {
            timerPulseStart[n] = micros();
        }
        else
        {
            timerPulseDuration[n] = micros() - timerPulseStart[n];
            timerPulseFinished[n] = true;
        }
    }

    /**
     * @brief isr handler for sensor0
     *
     */
    void IRAM_ATTR isrEcho0()
    {
        pulseEcho(0);
    }

    /**
     * @brief isr handler for sensor1
     *
     */
    void IRAM_ATTR isrEcho1()
    {
        pulseEcho(1);
    }

    /**
     * @brief isr handler for sensor2
     *
     */
    void IRAM_ATTR isrEcho2()
    {
        pulseEcho(2);
    }

    /**
     * @brief initialize the pinStates and attach interrupts
     *
     */
    void ultrasonicInit()
    {
        for (size_t i = 0; i < 3; i++)
        {
            pinMode(triggerPins[i], OUTPUT);
            pinMode(echoPins[i], INPUT);
        }
        attachInterrupt(echoPins[0], isrEcho0, CHANGE);
        attachInterrupt(echoPins[1], isrEcho1, CHANGE);
        attachInterrupt(echoPins[2], isrEcho2, CHANGE);
    }

    /**
     * @brief caculate distacne in cm from two-way time delta
     *
     * @param t delte t in micros
     * @return the equialent distance in cm
     */
    double microsToCm(unsigned long t)
    {
        return t / 58.0;
    }

    /**
     * @brief trigger a pusle signal on the ultrasonic sensor
     *
     * @param n the index of the sensor
     */
    void ultrasonicPulse(unsigned n)
    {
        // Sets the trigger on HIGH state for 10 micro seconds to send a series of pulses
        digitalWrite(triggerPins[n], LOW);
        delayMicroseconds(2);
        digitalWrite(triggerPins[n], HIGH);
        delayMicroseconds(10);
    }
}

/**
 * @brief background task for ultrasonic sensors.
 * updates sensor values in the background
 *
 * @param argument
 */
void ultrasonicTask(void *argument)
{
    Serial.print("ultrasonicTask is running on: ");
    Serial.println(xPortGetCoreID());
    ultrasonicInit();
    for (;;)
    {
        for (size_t i = 0; i < 3; i++)
        {
            ultrasonicPulse(i);
            while (timerPulseFinished[i] == false)
            {
                vTaskDelay(0);
            }
            timerPulseFinished[i] = false;
            usDistances[i] = microsToCm(timerPulseDuration[i]);
            Serial.printf("%d: %f ", i, usDistances[i]);
        }
        Serial.println();
        delay(1000);
    }
    Serial.println("ultrasonicTask closed");
    vTaskDelete(NULL);
}