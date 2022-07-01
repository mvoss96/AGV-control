#pragma once

/**
 * @brief background task for ultrasonic sensors.
 * updates sensor values in the background
 *
 * @param argument
 */
void ultrasonicTask(void *argument);

void ultrasonicPrint();

enum US_Sensors
{
    SENSOR_LEFT,
    SENSOR_FRONTL,
    SENSOR_FRONTC,
    SENSOR_FRONTR,
    SENSOR_RIGHT
};