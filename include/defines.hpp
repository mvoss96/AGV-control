#pragma once
/*!
 * Pin Definitions
 */
#define PIN_MOTOR_A1 26
#define PIN_MOTOR_A2 25
#define PIN_MOTOR_B1 32
#define PIN_MOTOR_B2 33

#define PIN_US0_TRIGGER MOTOR_PWM_A1 22
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
#define SERIAL_BAUDRATE 115200
#define MOTOR_PWM_FRQ 300000
#define MOTOR_PWM_RES 8
#define ENC_WHEEL_DIA 66.1
#define ENC_WHEEL_STEPS 20
#define WHEEL_DIST 150

#define MOTOR_PWM_A1 0
#define MOTOR_PWM_A2 1
#define MOTOR_PWM_B1 2
#define MOTOR_PWM_B2 3

/*!
 * helpers
 */

#define dbg(myFixedText, variableName)                   \
    Serial.print(F(#myFixedText " " #variableName "=")); \
    Serial.println(variableName);