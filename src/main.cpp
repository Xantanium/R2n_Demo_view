/*-------------------------- INCLUDES ----------------------------*/
////////////////////////////////////////////////////////////////////
/// Standard Includes
////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <TeensyThreads.h>
#include <Wire.h>

////////////////////////////////////////////////////////////////////
/// Custom Libraries
////////////////////////////////////////////////////////////////////
/// A. Component Drivers
#include <../API/Drivers/Controller/Xbox/Xbox.h>
#include <../API/Drivers/Sensors/IMU/BNO/BNO.h>
///
/// B. Mathematical Algorithms
#include <../API/Algos/Odometry/ThreeXYEncoder/ThreeXYEncoder.hpp>
#include <../API/Algos/PID/PID.h>
#include "Operations/TransferState/TransferState.h"
#include "Operations/Turret/Turret.h"
#include "core_pins.h"
#include "maths.h"
///
////////////////////////////////////////////////////////////////////
/// Robot Functionality
////////////////////////////////////////////////////////////////////
/// A. Data encapsulation structures
#include <Robot.h>
///
/// B. Hardware abstraction and meta
#include "ISR.h"
#include "button_maps.h"
#include "constants.h"
#include "pins_arduino.h"
#include "r2_pins.h"
///
/// C. Task Scheduler
#include <Scheduler/Scheduler.h>
///

/*-------------------------- OBJECTS -----------------------------*/
////////////////////////////////////////////////////////////////////
/// Standard Libraries
////////////////////////////////////////////////////////////////////
/// A. Threads
Threads::Mutex SerialLock;
///
////////////////////////////////////////////////////////////////////
/// Custom Libraries
////////////////////////////////////////////////////////////////////
/// A. Controller
USBHost myUsb;
USBHIDParser hid(myUsb);
BluetoothController bt(myUsb);
XboxController Ps3(myUsb);
///
/// B. IMU
rudra::BNO bno;
///
/// C. PID
PID pidLocoW(1.0f, 0.0f, 0.0f, 80.0f, 50.0f);
///
////////////////////////////////////////////////////////////////////
/// Robot Functionality
////////////////////////////////////////////////////////////////////
/// A. Scheduler
Scheduler scheduler;
///
/// B. Data Struct Instantiation
/**
 * @note
 * All of these objects are declared extern in their respective
 * headers. They must be initialized here for usage.
 */
Motion motion; ///< For Locomotion Control
Thrower thrower; ///< Throwing Control
PIDState pidState; ///< PID control data
Transfer transfer; ///< Transfer Control
Dribbler dribbler; ///< Dribbling Control
TurretYaw turret; ///< Turret Control
RelayState relayState; ///< For Relay Control
DefenceState defence; ///< Defence Control
OdometryState odom; ///< For Odometry Data Encapsulation
///

/*------------------------- FUNCTION -----------------------------*/
/*----------------------- DECLARATIONS ---------------------------*/
////////////////////////////////////////////////////////////////////
/// Robot Functionality
////////////////////////////////////////////////////////////////////
// A. Controller Handling
/**
 * @struct ControllerState
 * @brief Struct for controller methods encapsulation
 */
struct ControllerState {
    /**
     * @brief PS3 task
     * Sent as callback to the scheduler
     * @param ps3 reference to controller object.
     */
    static void controllerTask(XboxController &ps3);

    /**
     * @brief get left joystick values and put them
     * in motion X and motion Y
     */
    static void getLeftJoyVectors(XboxController &ps3);

    /**
     * @brief get right joystick values and put them
     * in motion W and turret velocity
     */
    static void getRightJoyVectors(XboxController &ps3);

    /**
     * @brief get Right trigger analog value
     * @return constrained value between the range
     * 0 to ELEV_MAX (constants.h)
     */
    static int getRightTrigger(XboxController &ps3);

    /**
     * @brief get Left trigger analog value
     *
     * @return constrained value between the range
     * 0 to ELEV_MAX (constants.h)
     */
    static int getLeftTrigger(XboxController &ps3);

    /**
     * @brief maps buttons to actions
     *
     * check `../include/button_maps.h`
     * for reference and details
     * about what each button does.
     *
     * @param ps3 controller object
     */
    static void getButtonMaps(XboxController &ps3);

private:
    /**
     * @brief Private helper function to fetch joystick values
     *
     * @param x X stick input (from JoyEnum)
     * @param y Y stick input (from JoyEnum)
     * @param xVal Where the x input should be put
     * @param yVal Where the y input should be put
     * @param ps3 Controller object
     */
    static void getJoyVectors(JoyEnum x, JoyEnum y, int &xVal, int &yVal, XboxController &ps3);
};
//
// B. Data Acquisition
void imuTask();
void lidarTask();
//
// C. Debug Task
void debugTask();
//
// D. Bot Init sequence
void botInit();
//
// E. Blink Example
void blinkTask();
//
/*----------------------- DRIVER CODE ----------------------------*/
////////////////////////////////////////////////////////////////////
/// Setup
////////////////////////////////////////////////////////////////////
///
void setup() {
    // Customary actions
    Serial.begin(115200);
    USBHost::begin();
    pinMode(LED_BUILTIN, OUTPUT);
    rudra::pinInit(); // pin initialization is done here for a cleaner setup
    digitalWrite(LED_BUILTIN, 1);

    // scheduler config
    scheduler.addTask([] { ControllerState::controllerTask(Ps3); }, 0);
    scheduler.addTask([] { relayState.relayTask(); }, 0);
    scheduler.addTask(imuTask, 10); // tasks defined in main can be passed directly without wrapper Lambda.
    scheduler.addTask([] { motion.drive(motion.stickX, motion.stickY, motion.stickW); }, 0);
    scheduler.addTask([] { turret.turretTask(); }, 10);
    scheduler.addTask([] { transfer.transferPipeline(); }, 10);
    scheduler.addTask(debugTask, 300);

    // Ready to run
    delay(1000);
    Serial.println("Initialized");
}
///
////////////////////////////////////////////////////////////////////
/// Loop
////////////////////////////////////////////////////////////////////
///
// Since the loop only runs the scheduler, it is safe to write testing code in the loop commenting the update() line.
// This may be useful for individual unit testing without leaving this file.
void loop() { scheduler.update(); }
///

/*------------------------- FUNCTION -----------------------------*/
/*------------------------ DEFINITIONS ---------------------------*/
////////////////////////////////////////////////////////////////////
/// Robot Functionality
////////////////////////////////////////////////////////////////////
// A. Controller
void ControllerState::getJoyVectors(const JoyEnum x, const JoyEnum y, int &xVal, int &yVal, XboxController &ps3) {
    xVal = static_cast<int>(map1(ps3.getAnalogHat(x), -32767, 32767, -128, 128));
    yVal = static_cast<int>(map1(ps3.getAnalogHat(y), -32767, 32767, -128, 128));

    if ((xVal > -20 && xVal < 20))
        xVal = 0;
    if ((yVal > -20 && yVal < 20))
        yVal = 0;
}
//
void ControllerState::controllerTask(XboxController &ps3) {
    ps3.joystickDataClear();
    myUsb.Task();

    getButtonMaps(Ps3);
    getLeftJoyVectors(Ps3);
    getRightJoyVectors(Ps3);
}
//
void ControllerState::getLeftJoyVectors(XboxController &ps3) {
    getJoyVectors(LeftHatX, LeftHatY, motion.stickX, motion.stickY, ps3);
}
//
void ControllerState::getRightJoyVectors(XboxController &ps3) {
    getJoyVectors(RightHatX, RightHatY, motion.stickW, turret.velocity, ps3);
}
//
int ControllerState::getRightTrigger(XboxController &ps3) { return ps3.getAnalogHat(T_TRANSFER_UP) / 255 * ELEV_MAX; }
int ControllerState::getLeftTrigger(XboxController &ps3) { return ps3.getAnalogHat(T_TRANSFER_DOWN) / 255 * ELEV_MAX; }
//
void ControllerState::getButtonMaps(XboxController &ps3) {
    if (ps3.getButtonClick(B_RELAY)) {
        relayState.isRelayOn = !relayState.isRelayOn;
        if (relayState.isRelayOn)
            relayState.doAtRelayOnce = true;
    }

    if (Ps3.getButtonClick(B_MPUSE)) {
        motion.mpUse = !motion.mpUse;
        turret.useJoyForMovement = motion.mpUse;
    }

    if (Ps3.getButtonClick(B_THROW_SPEED_INC)) {
        if (thrower.throwSpeed >= 4) {
            thrower.throwSpeed = 4;
            thrower.isThrowing = true;
        } else {
            thrower.throwSpeed++;
            thrower.isThrowing = true;
        }
    }

    if (Ps3.getButtonClick(B_THROW_SPEED_DEC)) {
        if (thrower.throwSpeed <= 0) {
            thrower.throwSpeed = 0;
            thrower.isThrowing = false;
        } else {
            thrower.throwSpeed--;
            thrower.isThrowing = true;
        }
    }

    if (Ps3.getButtonClick(B_CLAW_TOGGLE))
        dribbler.manual = true;
    if (Ps3.getButtonClick(B_INVERT))
        motion.invertAxis = !motion.invertAxis;
    if (Ps3.getButtonClick(B_MBACK_DIR))
        transfer.sendBallToDribble = !transfer.sendBallToDribble;
}
//
// B. Data Acquisition
void imuTask() {
    if (!Ps3.isConnected || !relayState.isRelayOn)
        return;
    odom.yaw = bno.yaw(BNO_WIRE);
}
//
// C. Debug Task
void debugTask() {
    // Threads::Scope scope(SerialLock);
    Serial.printf("A: %f, B: %f, C: %f\n", motion.maCap, motion.mbCap, motion.mcCap);
    Serial.printf("RelayState: %d\n", relayState.isRelayOn);
    Serial.printf("Turret velo: %d\n", turret.velocity);
    // Threads::yield();
}
//
// D. External Task Implementations
void RelayState::relayTask() {
    if (!Ps3.isConnected)
        return;
    digitalWrite(RELAY_PIN, relayState.isRelayOn);
    if (!isRelayOn) {
        rudra::safety();
        doAtRelayOnce = true;
        return;
    }

    if (doAtRelayOnce) {
        doAtRelayOnce = false;
        rudra::safety();
        bno.bnoInit(BNO_WIRE);
        delay(2000);
        rudra::safety();
    }
}
//
// E. Blink Example
void blinkTask() {
    static bool state = false;
    state = !state;
    digitalWrite(LED_BUILTIN, state);
}
//
// F. Turret Task
void TurretYaw::turretTask() const {
#define TURRET_MAX 60
    digitalWrite(TURRET_YAW_DIR, velocity > 0 ? 1 : 0);
    analogWrite(TURRET_YAW_PWM, round(abs((velocity / 128.0f) * TURRET_MAX)));
}
//
// G. Transfer Task
void Transfer::transferPipeline() {
    bool dir = false;
    if (ControllerState::getRightTrigger(Ps3) > 0)
        dir = TRANSFER_UP;
    else if (ControllerState::getLeftTrigger(Ps3) > 0)
        dir = !TRANSFER_UP;

    digitalWrite(DRIBBLE_TO_FEED_DIR, dir);

    analogWrite(
            DRIBBLE_TO_FEED_PWM, dir ? ControllerState::getRightTrigger(Ps3) : ControllerState::getLeftTrigger(Ps3));
}
//
/*---------------------------- ISR -------------------------------*/
/*------------------------ DEFINITIONS ---------------------------*/
void ISR_X_ENCODER() { }
void ISR_YL_ENCODER() { }
void ISR_YR_ENCODER() { }
void ISR_DRIBBLE_LASER() { }
void ISR_TURRET() { }
