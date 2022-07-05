#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "defines.hpp"
#include "object_recognition.hpp"
#include "telnet_debug.hpp"

namespace
{
    Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);
    unsigned CO_OBJ_THRESHOLD = 0;
}

bool colorSensorInit()
{
    digitalWrite(PIN_LED_COLOR, HIGH);
    Wire.begin(PIN_SDA_COLOR, PIN_SCL_COLOR);
    pinMode(PIN_LED_COLOR, OUTPUT);
    return tcs.begin();
}

unsigned returnLux()
{
    uint16_t r, g, b, c, lux, lux_mean;
    tcs.getRawData(&r, &g, &b, &c);
    lux_mean = tcs.calculateLux(r, g, b);

    long time = millis();
    for (size_t i = 0; i < NUM_CO_SAMPLES; i++)
    {
        tcs.getRawData(&r, &g, &b, &c);
        lux = tcs.calculateLux(r, g, b);
        lux_mean += lux;
        // Serial.print(r);
        // Serial.print(" ");
        // Serial.print(g);
        // Serial.print(" ");
        // Serial.print(b);
        // Serial.println();
        delay(100);
    }
    lux_mean = lux_mean / NUM_CO_SAMPLES;
    DEBUG_VAR(lux_mean);
    return lux_mean;
}

void calibrateLux()
{
    digitalWrite(PIN_LED_COLOR, LOW);
    delay(100);
    unsigned lux_mean = returnLux();
    digitalWrite(PIN_LED_COLOR, HIGH);
    DEBUG_MSG("calibrate baseline lux to:");
    DEBUG_VAR(lux_mean);
    CO_OBJ_THRESHOLD = lux_mean;
}

unsigned measureObject()
{
    uint16_t r_mean, g_mean, b_mean;
    uint16_t r, g, b, c, colorTemp, lux;

    digitalWrite(PIN_LED_COLOR, HIGH);

    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    lux = tcs.calculateLux(r, g, b);

    r_mean = r;
    g_mean = g;
    b_mean = b;

    long time = millis();

    for (size_t i = 0; i < NUM_CO_SAMPLES; i++)
    {
        tcs.getRawData(&r, &g, &b, &c);
        r_mean += r;
        g_mean += g;
        b_mean += b;
        delay(100);
    }
    r_mean = r_mean / NUM_CO_SAMPLES;
    g_mean = g_mean / NUM_CO_SAMPLES;
    b_mean = b_mean / NUM_CO_SAMPLES;

    if ((r_mean * 100 / b_mean) < 72 && (g_mean * 100 / b_mean) < 90)
    {
        return RECOGNITION_COTTON;
    }
    else if ((r_mean * 100 / b_mean) > 72 && (r_mean * 100 / b_mean) < 180 && (g_mean * 100 / b_mean) > 90 && (g_mean * 100 / b_mean) > 165)
    {
        return RECOGNITION_GUMMY;
    }
    else if ((r_mean * 100 / b_mean) > 180 && (g_mean * 100 / b_mean) > 165)
    {
        return RECOGNITION_BALL;
    }
    // double cheking if none where true which is the most likely
    else if ((r_mean * 100 / b_mean) < 50)
    {
        return RECOGNITION_COTTON;
    }
    else if ((r_mean * 100 / b_mean) > 220)
    {
        return RECOGNITION_BALL;
    }
    else if ((r_mean * 100 / b_mean) > 50)
    {
        return RECOGNITION_GUMMY;
    }
    else
    {
        return RECOGNITION_ERROR;
    }
}

bool objectLoaded()
{
    DEBUG_VAR(CO_OBJ_THRESHOLD);
    digitalWrite(PIN_LED_COLOR, LOW);
    delay(100);
    unsigned lux_mean = returnLux();
    digitalWrite(PIN_LED_COLOR, HIGH);
    if (lux_mean > CO_OBJ_THRESHOLD)
        return false;
    else
        return true;
}