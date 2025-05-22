/**
 * @file CitadelMainMCU.h
 * @author your name (you@domain.com)
 * @brief
 */
#pragma once

//------------------------------------------------------------------------------------------------//
//   DOIT ESP32 Devkit V1 (URC 2025, Citadel V3 D:)
//------------------------------------------------------------------------------------------------//

#define CAN_TX 13
#define CAN_RX 14

#define PIN_VDIV_BATT 39
#define PIN_VDIV_12V 36
#define PIN_VDIV_5V 34
// No 3.3V VDC

#define PIN_STEP_1 25
#define PIN_STEP_2 32
#define PIN_STEP_3 4
#define PIN_STEP_4 19
#define PIN_STEP_DIR 33  // Direction pin for all steppers
#define PIN_STEP_ENABLE 27  // Enable pin for all steppers (Why motors get so hot??)

#define motorInterfaceType 1

#define PIN_VIBMOTOR 18
#define PIN_FAN_1 23
#define PIN_FAN_2 22
#define PIN_FAN_3 21

#define PIN_PWM_SERVO_1 26  // Need to figure out if we need to use one PWM for all or individual PWM lines
#define PIN_PWM_SERVO_2 5
#define PIN_PWM_SERVO_3 15
