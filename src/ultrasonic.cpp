#include <Arduino.h>
#include "defines.hpp"
#include "ultrasonic.hpp"

// stores measured distances form ultrasonic sensors for use by other tasks
double usDistances[NUM_SENSORS] = {0};

// used to enable/disable the ultrasonic routine
bool ultrasonicEnable = true;
bool ultrasonicStarted = false;

/**
 * @brief private area
 *
 */
namespace
{
    volatile unsigned long timerPulseStart[NUM_SENSORS] = {0};
    volatile unsigned long timerPulseDuration[NUM_SENSORS] = {0};
    volatile bool timerPulseFinished[NUM_SENSORS] = {0};
    const unsigned triggerPins[NUM_SENSORS] = {PIN_US0_TRIGGER, PIN_US1_TRIGGER, PIN_US2_TRIGGER, PIN_US3_TRIGGER, PIN_US4_TRIGGER};
    const unsigned echoPins[NUM_SENSORS] = {PIN_US0_ECHO, PIN_US1_ECHO, PIN_US2_ECHO, PIN_US3_ECHO, PIN_US4_ECHO};

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
     * @brief isr handler for sensor3
     *
     */
    void IRAM_ATTR isrEcho3()
    {
        pulseEcho(3);
    }

    /**
     * @brief isr handler for sensor3
     *
     */
    void IRAM_ATTR isrEcho4()
    {
        pulseEcho(4);
    }

    /**
     * @brief initialize the pinStates and attach interrupts
     *
     */
    void ultrasonicInit()
    {
        ultrasonicEnable = true;
        for (size_t i = 0; i < NUM_SENSORS; i++)
        {
            pinMode(triggerPins[i], OUTPUT);
            pinMode(echoPins[i], INPUT);
        }
        attachInterrupt(echoPins[0], isrEcho0, CHANGE);
        attachInterrupt(echoPins[1], isrEcho1, CHANGE);
        attachInterrupt(echoPins[2], isrEcho2, CHANGE);
        attachInterrupt(echoPins[3], isrEcho3, CHANGE);
        attachInterrupt(echoPins[4], isrEcho4, CHANGE);
    }

    /**
     * @brief caculate distacne in cm from two-way time delta
     *
     * @param t delte t in micros
     * @return the equialent distance in cm
     */
    double microsToCm(unsigned long t)
    {
        double r = t / 58.0;
        return (r > US_MAX_DIST) ? US_MAX_DIST : r;
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

void ultrasonicPrint()
{
    for (size_t i = 0; i < NUM_SENSORS; i++)
    {
        Serial.printf("%d: %f ", i, usDistances[i]);
    }
    Serial.println();
}

void ultrasonicTask(void *argument)
{
    Serial.print("ultrasonicTask is running on: ");
    Serial.println(xPortGetCoreID());
    ultrasonicInit();
    for (;;)
    {
        while (ultrasonicEnable == false)
        {
            vTaskDelay(100);
        }
        for (size_t i = 0; i < NUM_SENSORS; i++)
        {
            ultrasonicPulse(i);
            while (timerPulseFinished[i] == false)
            {
                vTaskDelay(0);
            }
            ultrasonicStarted = true;
            //Serial.printf("us %i\n", i);
            timerPulseFinished[i] = false;
            usDistances[i] = microsToCm(timerPulseDuration[i]);
        }
        ultrasonicPrint();
        delay(500);
    }
    Serial.println("ultrasonicTask closed");
    vTaskDelete(NULL);
}