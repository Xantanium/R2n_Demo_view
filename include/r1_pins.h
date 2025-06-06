#pragma once
#include <Arduino.h>
#include "constants.h"
#include "core_pins.h"

#ifdef R1_pins

// NOTE: A lot of this code is reduntant now, since many of these are removed from the robot.

// READ THE BELOW COMMENT, PLEASE
/*
README README README README README README README README README README README README README
******************************************************************************************

ELECTRONICS PERSON:
    > put Pins in front of the Pin Names.

        e.g:

        #define DIR_A 4
        #define BNO_WIRE Wire1
        #define BLDC_SERIAL Serial8

    > after a pin is written, uncomment the pinMode() line of the respective pin
      from `pinInit()`.
      If the pin is motor/actuator, uncomment respective digitalWrite() in `safety()`.

        e.g:

        #define PWM_A 5
        -> uncomment pinMode(PWM_A, OUTPUT);
        in this case since it's a motor:
        -> uncomment digitalWrite(PWM_A, 0);
           in safety().
        (uncommenting is removing the '//')


    p.s please don't touch macros

******************************************************************************************
README README README README README README README README README README README README README
*/

// RELAY
#define RELAY_PIN 12
#define R1_pins
// LOCOMOTORS
#ifndef R1_pins
#define DIR_A 0
#define DIR_B 2
#define DIR_C 6

#define PWM_A 1
#define PWM_B 3
#define PWM_C 7
#else
#define DIR_A 0
#define DIR_B 4
#define DIR_C 6
#define DIR_D 2

#define PWM_A 1
#define PWM_B 5
#define PWM_C 7
#define PWM_D 3
#endif

/* --------------------------------- MECHANISMS --------------------------------- */

// TRANSFER / ELEVATOR / FEEDING
// (magchi motor)
#define DEFENCE_TO_FEED_DIR 41
#define DEFENCE_TO_FEED_PWM 15

// (elevator belt motor)
#define DRIBBLE_TO_FEED_DIR 40
#define DRIBBLE_TO_FEED_PWM 14

// DOOR
// #define DOOR_SOLENOID 23

// CLAW
// #define DRIBBLE_ACTUATOR 22

// EXTENSION
// #define EXTENSION_ACTUATOR 21

// DEFENCE
// (motors viewed from the back)
// #define DEFENCE_LEFT_DIR 20
#define DEFENCE_RIGHT_PWM 18

#define DEFENCE_LEFT_PWM 19

// TURRET
#define TURRET_YAW_DIR 8
#define TURRET_YAW_PWM 9
// #define TURRET_YAW_LIM 2482

// BREAKOUT
#define TURRET_ENC_A 11
#define TURRET_ENC_B 10

// THROWER
#define THROW_DIR 12 // electronically high
#define THROW_PWM 24
/* ---------------------------------- SENSORS ----------------------------------- */

// LASERS
// #define BUMP_LASER 378 // mid stopping laser
#define DRIBBLE_LASER 30 // claw laser
// #define DEFENCE_TO_FEEDER_LASER 349 // ball catcher

// IR
#define LINE_IR

// ENCODERS
// (refer top view) : parallel to horizontal = x, vertical = y (Left and Right)
#define X_ENC_A 21
#define X_ENC_B 20

#define YL_ENC_A 23
#define YL_ENC_B 22

// #define YR_ENC_A 282
// #define YR_ENC_B 294

/* ----------------------------------- WIRES ------------------------------------ */

// BNO
#define BNO_WIRE Wire

// ESP
#define ESP_WIRE Wire

/* ----------------------------------- UARTS ------------------------------------ */

// LIDAR
// (LR as seen from back)
#define LIDAR_LEFT_SERIAL Serial8
#define LIDAR_RIGHT_SERIAL Serial7

// PITCH SERVO
#define SERVO_SERIAL Serial4

// BLDC
#ifdef BLDC
#define BLDC_SERIAL Serial4
#endif
/* ----------------------------------- MACROS ------------------------------------ */

// DigWrit
#define SET_PIN(pin, val)                                                                                              \
    ({                                                                                                                 \
        val == LOW ? *portOutputRegister(digitalPinToPort((pin))) &= ~digitalPinToBitMask((pin))                       \
                   : *portOutputRegister(digitalPinToPort((pin))) |= digitalPinToBitMask((pin));                       \
    })

/* ---------------------------------- FUNCTIONS ----------------------------------- */
namespace rudra {
    void pinInit();
    void safety();
} // namespace rudra

/**
 *                      XXX                                XXX
 *                    X     X                            X     X
 *                   X XXXXX X                          X XXXXX X
 *                  XXX     XXX                        XXX     XXX
 *                 X           XXXXXXXXXXXXXXXXXXXXXXXX           X
 *                X             X                    X             X
 *               X               X       SONY       X               X
 *               X               X  XXX        XXX  X               X
 *               X               X        XXX       X               X
 *                X           XXXXXXX    X   X   XXXXXXX           XX
 *                XXXX       XXX     X    XXX   X     XXX       XXX X
 *               X    XXXXXXX         X        X         XXXXXXX     X
 *               X          XX       XXXXXXXXXXXX       X            X
 *              X          X  XXXXXXX            XXXXXXX XX           X
 *             X          X                                X          X
 *             X        XX                                  X          X
 *            X        X                                     XX        X
 *            XXX   XXX                                        XX    XXX
 *              XXXXX                                           XXXXXX
 */
#endif R1_pins
