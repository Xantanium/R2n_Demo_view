//
// Created by xantanium on 19/5/25.
//
#include "Motion.h"
#include "constants.h"
#ifndef R1_pins
#include "r2_pins.h"
#else
#include "r1_pins.h"
#endif
#include "wiring.h"


/// @brief Called when robot to be driven. Resolves axes inversion, argument normalization and calls 'getSpeed' and
/// 'setMotion'
/// @param argX velocity in x
/// @param argY velocity in y
/// @param argW velocity in w
void Motion::drive(const int argX, const int argY, const int argW) {

    motion.valX = motion.invertAxis ? (-argX / 128.0f) : (argX / 128.0f);
    motion.valY = motion.invertAxis ? (argY / 128.0f) : (-argY / 128.0f);
    motion.valW = argW / 128.0f;

    getSpeed();
    setMotion();
}

/// @brief Calculates PWM values to be written to each wheel.
void Motion::getSpeed() {
#ifndef R1_pins
    motion.maSpeed = ((motion.valX * (0.667f)) + (motion.valY * (0.000f)) + (motion.valW * 0.333f));
    motion.mbSpeed = ((motion.valX * (-0.333f)) + (motion.valY * (-0.577f)) + (motion.valW * 0.333f));
    motion.mcSpeed = ((motion.valX * (-0.333f)) + (motion.valY * (0.577f)) + (motion.valW * 0.333f));

    motion.maCap = constrain(round(abs(motion.maSpeed) * MAX_PWM), 0, 255);
    motion.mbCap = constrain(round(abs(motion.mbSpeed) * MAX_PWM), 0, 255);
    motion.mcCap = constrain(round(abs(motion.mcSpeed) * MAX_PWM), 0, 255);
#else
    motion.maSpeed = (motion.valX * -0.33f) + (motion.valY * -0.33f) + (motion.valW * -0.33f);
    motion.mbSpeed = (motion.valX * -0.33f) + (motion.valY * 0.33f) + (motion.valW * -0.33f);
    motion.mcSpeed = (motion.valX * 0.33f) + (motion.valY * -0.33f) + (motion.valW * -0.33f);
    motion.mdSpeed = (motion.valX * 0.33f) + (motion.valY * 0.33f) + (motion.valW * -0.33f);

    motion.maCap = round(abs(motion.maSpeed) * MAX_PWM);
    motion.mbCap = round(abs(motion.mbSpeed) * MAX_PWM);
    motion.mcCap = round(abs(motion.mcSpeed) * MAX_PWM);
    motion.mdCap = round(abs(motion.mdSpeed) * MAX_PWM);

#endif
}

/// @brief Writes direction and pwm to motors. Uses global variables modified by getSpeed to do so.
void Motion::setMotion() {
    // NOTE: only use once all pins are defined

#ifndef R1_pins

    digitalWrite(DIR_A, motion.maSpeed < 0 ? 0 : 1);
    digitalWrite(DIR_B, motion.mbSpeed < 0 ? 0 : 1);
    digitalWrite(DIR_C, motion.mcSpeed < 0 ? 0 : 1);

    analogWrite(PWM_A, motion.maCap);
    analogWrite(PWM_B, motion.mbCap);
    analogWrite(PWM_C, motion.mcCap);

#else
    digitalWrite(DIR_A, motion.maSpeed > 0 ? 0 : 1);
    digitalWrite(DIR_B, motion.mbSpeed > 0 ? 0 : 1);
    digitalWrite(DIR_C, motion.mcSpeed > 0 ? 0 : 1);
    digitalWrite(DIR_D, motion.mdSpeed > 0 ? 0 : 1);

    analogWrite(PWM_A, motion.maCap);
    analogWrite(PWM_B, motion.mbCap);
    analogWrite(PWM_C, motion.mcCap);
    analogWrite(PWM_D, motion.mdCap);

#endif
}
