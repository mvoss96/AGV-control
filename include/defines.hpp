#pragma once
/*!
 * Pin Definitions
 */
#define PIN_STEPPER_L_STEP 26
#define PIN_STEPPER_L_DIR 25
#define PIN_STEPPER_L_SLEEP 14
#define PIN_STEPPER_R_STEP 33
#define PIN_STEPPER_R_DIR 32
#define PIN_STEPPER_R_SLEEP 27

#define PIN_US0_TRIGGER 4
#define PIN_US0_ECHO 34
#define PIN_US1_TRIGGER 22
#define PIN_US1_ECHO 18
#define PIN_US2_TRIGGER 19
#define PIN_US2_ECHO 5
#define PIN_US3_TRIGGER 23
#define PIN_US3_ECHO 17
#define PIN_US4_TRIGGER 16
#define PIN_US4_ECHO 35
#define NUM_SENSORS 5

/*!Tag Configuration Settings */
#define TAG_CENTER 600
#define TAG_CENTER_DEADZONE_SMALL 50
#define TAG_CENTER_DEADZONE 100
#define TAG_CLOSE_SIZE 160
#define TAG_SEARCH_TIMEOUT 20000
#define TAG_LAST_SEEN_TIMEOUT 1000
#define REPOSITION_MAX_STOPS 5
#define REPOSITION_MAX_TIME 10000

/*wifi Configuration Settings */
#define WIFI_SSID "AGV1"
#define TELNET_PORT 23
#define UDP_PORT 7709
#define UDP_COMM_PORT 7708
#define UDP_TIMEOUT 1000
#define SERIAL_BAUDRATE 115200

/*!Stepper Motor Configuration Settings */
#define STEPER_STEPS_PER_ROT 2048
#define WHEEL_ROTS_360 2.75
#define STEPS_360 (STEPER_STEPS_PER_ROT * WHEEL_ROTS_360)
#define STEPS_90 (STEPS_360 * 0.25)
#define STEPS_45 (STEPS_90 * 0.5)
#define STEPPER_MAX_RPM 20
#define STEPPER_TURN_RPM 5
#define STEPPER_SLOW_TURN_RPM 3

/*!telnet setings */
#define DEBUG_ON 1
#define DEBUG_USE_SERIAL 1
#define DEBUG_USE_TELNET 1

/*!ultrasonic settings */
#define US_MAX_DIST 200
#define US_MIN_TRIGGER 7
#define US_BASE_TRIGGER 5
#define US_NEAR_TRIGGER 15

/*!
 * helpers
 */

enum missions
{
    NO_MISSION,
    DELIVER,
    GET_GUMMY,
    GET_COTTON,
    GET_BALL
};

#define dbg(myFixedText, variableName)                   \
    Serial.print(F(#myFixedText " " #variableName "=")); \
    Serial.println(variableName);