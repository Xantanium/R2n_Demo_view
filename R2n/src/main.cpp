/*-------------------------- INCLUDES ----------------------------*/
////////////////////////////////////////////////////////////////////
/// Standard Includes
////////////////////////////////////////////////////////////////////
///
#include <Arduino.h>
#include <TeensyThreads.h>
#include <Wire.h>
#include "Algos/Odometry/Primitives.h"
#include "Algos/Projectile/ShooterData.h"
#include "core_pins.h"
///
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
#include <../API/Algos/Projectile/Projectile.h>
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
#ifdef R1_pins
#include "r1_pins.h"
#else
#include "r2_pins.h"
#endif // R1
///
/// C. Task Scheduler
#include <Scheduler/Scheduler.h>
///

/*-------------------------- OBJECTS -----------------------------*/
////////////////////////////////////////////////////////////////////
/// Standard Libraries
////////////////////////////////////////////////////////////////////
/// A. Threads
// Scoped mutexes to be used, manual locking is boring af
///
Threads::Mutex SerialLock;
Threads::Mutex OdomLock;
Threads::Mutex PIDLock;
Threads::Mutex ShooterLock;
Threads::Mutex TurretLock;
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
PID pidTurret(1.0f, 0.0f, 0.0f, 80.0f, 50.0f);
///
/// D. EncoderOdometry
rudra::ThreeXYEncoder::PPRs pprs(600, 600, 600);
rudra::ThreeXYEncoder encOdom(510.40f, 47.3f, pprs);
rudra::Pose botPose(0, 0, 0);
long int xCount { 0 }, yLCount { 0 }, yRCount { 0 };
///
/// E. Projectile
rudra::ShooterData shooter;
rudra::Projectile projectile;

long int turretCount;
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
// F. PID loop
void PIDTask();
//
// G. Resets and customary
void resetTask_threaded();
//
// H. Odometry Task
void odomTask();
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
    BNO_WIRE.begin();
    bno.bnoInit(BNO_WIRE);
    // scheduler config
    /**
     * Task Order:
     * Acquire from controller
     * -> handle relay
     * -> get data from sensors
     * -> calculate control and odom
     * -> Locomotion
     * -> Mechanism handling
     * -> Serial debug
     * -> loop resets (can be threaded)
     */
    scheduler.addTask([] { ControllerState::controllerTask(Ps3); }, 0);
    scheduler.addTask([] { relayState.relayTask(); }, 0);
    scheduler.addTask([] { imuTask(); }, 10);
    scheduler.addTask([] { PIDTask(); }, 10);
    scheduler.addTask([] { odomTask(); }, 10);
    scheduler.addTask(
            [] {
                Threads::Scope pidScope(PIDLock);
                motion.drive(motion.stickX, motion.stickY, motion.mpUse ? pidState.locoW : motion.stickW);
            },
            0);
    scheduler.addTask([] { turret.turretTask(); }, 10);
    scheduler.addTask([] { transfer.transferPipeline(); }, 10);
    scheduler.addTask([] { debugTask(); }, 300);
    scheduler.addTask([] { resetTask_threaded(); }, 0);

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
        if (turret.runManually)
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
    // if (Ps3.getButtonClick(B_MBACK_DIR))
    //     transfer.sendBallToDribble = !transfer.sendBallToDribble;

    if (Ps3.getButtonClick(B_AUTO_TURRET)) {
        turret.runManually = !turret.runManually;
    }
}
//
// B. Data Acquisition
void imuTask() {
    Threads::Scope odomScope(OdomLock);
    odom.yaw = bno.yaw(BNO_WIRE);
}
//
// C. Debug Task
void debugTask() {
    Threads::Scope scope(SerialLock);
    Serial.printf("A: %f, B: %f, C: %f\n", motion.maCap, motion.mbCap, motion.mcCap);
    Serial.printf("RelayState: %d\n", relayState.isRelayOn);
    Serial.printf("Turret velo: %d\n", turret.velocity);
    Threads::Scope odomScope(OdomLock);
    Serial.printf("Yaw: %f, LocoW: %f\n", odom.yaw, pidState.locoW);
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
void TurretYaw::turretTask() {
    /**
     * This task handles angling the turret.
     * Turret can be
     * a. manual : where joystick is used for movement
     * b. automatic : where odometry data is used for movement.
     *
     * In manual; aiming, speed setting and projectile angle setting
     * is done by the operator, whereas in automatic teensy handles
     * this on its own. These functions/classes are of help:
     *
     * 1.ThreeXYEncoder -> gets the current Pose of the robot.
     * 2. Projectile -> has several helpers:
     * 									1.
     */
    if (runManually) {
        moveTurret(velocity);
    } else {
        Threads::Scope turretScope(TurretLock);
        Threads::Scope shooterScope(ShooterLock);
        Threads::Scope PIDScope(PIDLock);
        turret.setpoint = shooter.turretAngle;
        moveTurret(pidState.turretOut);
    }
}
//
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
// E. Blink Example
void blinkTask() {
    static bool state = false;
    state = !state;
    digitalWrite(LED_BUILTIN, state);
}
//
// F. PID Task
void PIDTask() {
    float yawError = 0, turretError = 0;
    {
        Threads::Scope odomScope(OdomLock);
        odom.wSetpoint = odom.yaw > 179.0f ? 359.0f : 0.0f;
        yawError = odom.wSetpoint - odom.yaw;
    }

    yawError = yawError < -179 ? (yawError + 360) : (yawError > 179 ? (yawError - 360) : (yawError));

    {
        Threads::Scope turretScope(TurretLock);
        turretError = turret.setpoint - turret.angle;
    }

    Threads::Scope PIDScope(PIDLock);
    pidState.locoW = pidLocoW.get_pid(yawError, 2);
    pidState.turretOut = pidTurret.get_pid(turretError, 1);
}
//
// G. Resets
void resetTask_threaded() {
    bool shouldReset = false;
    {
        Threads::Scope odomScope(OdomLock);
        shouldReset = abs(odom.yaw - odom.wSetpoint) < 0.5f;
    }

    if (shouldReset) {
        Threads::Scope PIDScope(PIDLock);
        pidState.locoW = 0.0f;
        pidLocoW.reset_I();
    }
    /*OdomLock.lock();
    if (abs(odom.yaw - odom.wSetpoint) < 5.0f) {
        PIDLock.lock();
        pidState.locoW = 0.0f;
        pidLocoW.reset_I();
        PIDLock.unlock();
    }
    OdomLock.unlock();*/
}
//
// H. Odom Task
void odomTask() {
    Threads::Scope odomScope(OdomLock);
    // This Bot pose is acquired wrt. arena, and will be used for hoop automation
    botPose = encOdom.update(xCount, yLCount, yRCount, odom.yaw);
    Threads::Scope shooterScope(ShooterLock);
    // shooter instance is populated with actual values
    projectile.getProjectile(shooter, botPose);
}
//
/*---------------------------- ISR -------------------------------*/
/*------------------------ DEFINITIONS ---------------------------*/
void ISR_X_ENCODER() {
#ifdef R1_pins // Locomotion Encoders only exist on R1 at this point hence encoder pins aren't `well` defined on R2.
    digitalRead(X_ENC_B) ? xCount++ : xCount--;
#endif // R1_pins
}
void ISR_YL_ENCODER() {
#ifdef R1_pins
    digitalRead(YL_ENC_B) ? yLCount++ : yLCount--;
#endif
}
void ISR_YR_ENCODER() { }
void ISR_DRIBBLE_LASER() { }
void ISR_TURRET() { digitalRead(TURRET_ENC_B) ? turretCount++ : turretCount--; }
