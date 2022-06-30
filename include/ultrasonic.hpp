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
    SENSOR_FRONT,
    SENSOR_RIGHT
};