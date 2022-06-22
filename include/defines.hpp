#pragma once
/*!
 * Pin Definitions
 */
#if 0
/*!DC-Motor */
#define PIN_MOTOR_A1 26
#define PIN_MOTOR_A2 25
#define PIN_MOTOR_B1 32
#define PIN_MOTOR_B2 33
#endif

/*!Stepper Motor */
#define STEPPER_L_STEP 26
#define STEPPER_L_DIR 25
#define STEPPER_L_SLEEP 14
#define STEPPER_R_STEP 33
#define STEPPER_R_DIR 32
#define STEPPER_R_SLEEP 27

#define PIN_US0_TRIGGER 22
#define PIN_US0_ECHO 18
#define PIN_US1_TRIGGER 19
#define PIN_US1_ECHO 5
#define PIN_US2_TRIGGER 23
#define PIN_US2_ECHO 17

#define PIN_ENC_A 35
#define PIN_ENC_B 34

/*!
 * Configuration Settings
 */
#define TAG_CENTER 600
#define TAG_CENTER_DEADZONE 100
#define WIFI_SSID "AGV1"
#define UDP_PORT 7709
#define UDP_TIMEOUT 1000
#define SERIAL_BAUDRATE 115200
#define ENC_WHEEL_DIA 66.1
#define ENC_WHEEL_STEPS 20
#define WHEEL_DIST 150

#if 0
/*!DC-motors */
#define MOTOR_PWM_A1 0
#define MOTOR_PWM_A2 1
#define MOTOR_PWM_B1 2
#define MOTOR_PWM_B2 3
#define MOTOR_PWM_FRQ 300000
#define MOTOR_PWM_RES 8
#endif

/*!Stepper Motor */
#define STEPER_STEPS_PER_ROT 2048
#define STEPPER_MAX_RPM 20

/*!
 * helpers
 */

#define dbg(myFixedText, variableName)                   \
    Serial.print(F(#myFixedText " " #variableName "=")); \
    Serial.println(variableName);