#include "constants.h"
#ifdef R1_pins
#include "r1_pins.h"
namespace rudra {
    void pinInit() {
        /* ------------------- OUTPUT SET ------------------- */

        // LOCOMOTORS
        pinMode(DIR_A, OUTPUT);
        pinMode(DIR_B, OUTPUT);
        pinMode(DIR_C, OUTPUT);

        pinMode(PWM_A, OUTPUT);
        pinMode(PWM_B, OUTPUT);
        pinMode(PWM_C, OUTPUT);

        // TRANSFER / ELEVATOR / FEEDER
        pinMode(DEFENCE_TO_FEED_DIR, OUTPUT);
        pinMode(DEFENCE_TO_FEED_PWM, OUTPUT);
        pinMode(DRIBBLE_TO_FEED_DIR, OUTPUT);
        pinMode(DRIBBLE_TO_FEED_PWM, OUTPUT);

        // RELAY
        pinMode(RELAY_PIN, OUTPUT);

        // pinMode(THROW_DIR, OUTPUT);
        pinMode(THROW_PWM, OUTPUT);

        // DOOR
        // pinMode(DOOR_SOLENOID, OUTPUT);

        // CLAW
        // pinMode(DRIBBLE_ACTUATOR, OUTPUT);

        // EXTENSION
        // pinMode(EXTENSION_ACTUATOR, OUTPUT);

        // DEFENCE
        // pinMode(DEFENCE_LEFT_DIR, OUTPUT);
        // pinMode(DEFENCE_RIGHT_PWM, OUTPUT);

        // pinMode(DEFENCE_LEFT_PWM, OUTPUT);

        // TURRET
        pinMode(TURRET_YAW_DIR, OUTPUT);
        pinMode(TURRET_YAW_PWM, OUTPUT);

        /* ------------------- INPUT SET ------------------- */

        // LASERS
        // pinMode(BUMP_LASER, INPUT_PULLUP);
        pinMode(DRIBBLE_LASER, INPUT_PULLUP);
        // pinMode(DEFENCE_TO_FEEDER_LASER, INPUT_PULLUP);

        // ENCODERS
        pinMode(X_ENC_A, INPUT_PULLUP);
        pinMode(X_ENC_B, INPUT_PULLUP);

        pinMode(YL_ENC_A, INPUT_PULLUP);
        pinMode(YL_ENC_B, INPUT_PULLUP);

        // pinMode(YR_ENC_A, INPUT_PULLUP);
        // pinMode(YR_ENC_B, INPUT_PULLUP); */

        pinMode(TURRET_ENC_A, INPUT_PULLUP);
        pinMode(TURRET_ENC_B, INPUT_PULLUP);
        // LIMIT SWITCHES

        /* ----------------- ANALOG FREQ ------------------- */
        analogWriteFrequency(PWM_A, PWM_FREQUENCY);
        analogWriteFrequency(PWM_B, PWM_FREQUENCY);
        analogWriteFrequency(PWM_C, PWM_FREQUENCY);

        analogWriteFrequency(DEFENCE_TO_FEED_PWM, PWM_FREQUENCY);
        analogWriteFrequency(DRIBBLE_TO_FEED_PWM, PWM_FREQUENCY);
        analogWriteFrequency(TURRET_YAW_PWM, PWM_FREQUENCY);

        analogWriteFrequency(THROW_PWM, PWM_FREQUENCY);
    }

    void safety() {
        digitalWrite(PWM_A, 0);
        digitalWrite(PWM_B, 0);
        digitalWrite(PWM_C, 0);

        digitalWrite(DEFENCE_LEFT_PWM, 0);
        digitalWrite(DEFENCE_RIGHT_PWM, 0);

        digitalWrite(DEFENCE_TO_FEED_PWM, 0);
        digitalWrite(DRIBBLE_TO_FEED_PWM, 0);

        digitalWrite(THROW_PWM, 0);

        // digitalWrite(DOOR_SOLENOID, 0);
        // digitalWrite(DRIBBLE_ACTUATOR, 0);
        // digitalWrite(EXTENSION_ACTUATOR, 0);

        digitalWrite(THROW_PWM, 0);
        digitalWrite(TURRET_YAW_PWM, 0);
    }
}
#endif // R1_Pins
