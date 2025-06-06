# Tasks Of the Robot:

Preface:
The basketballl playing robot consists of following mechanisms:
1. Three Wheel Holonomic drive chassis
2. Pneumatic actuator claw mechanism for dribbling
3. A Defence mechanism consisting of a net to catch the incoming ball
4. A transfer chamber to take the ball to dribbler claw or to the shooter turret mounted on top of the robot.
5. Turret mechanism to change the yaw of the throwing mechanism
6. Single motor throwing mechanism
7. Servo operated projectile angle changer for throwing mechanism.

On the manual robot, the Ps3/Xbox controller sends data to the usb dongle which is read by teensy 4.1
Upon certain controller events the bot performs certain actions. The local sensors on the robot are read constantly,
updating the positional information of the robot, which is further used to calculate certain metrics-
Throwing speed, thrower pitch angle, turret yaw angle, etc.

There are following types of tasks:

## 1. Interrupt triggered:

These tasks are performed based on hardware interrupts to teensy 4.1. These do not need scheduling.
1. Encoder reading
2. Laser reading (triggers upon proximity of the object obstructing the light)
3. IR reading (triggered upon sensing white line on the arena, resets 3 vars)

## 2. Interval Tasks:

These are performed periodically, so can be scheduled based on the Interval triggers.
Currently, these are done using millis() polling every loop. Considering to shift them to 
IntervalTimer objects to ensure consistent timing between samples.

1. I2C read from bno (every 10 ms for consistent sampling)
2. PID control (every 10 ms, since dt for pid loop = 10 ms.)
3. Serial Debug (every 300 ms, only for testing phase.) // won't shift to interval timer since very low priority.
4. SD Card datalog (every 100 ms, for post fault debugging)

## 3. Controller Tasks

These include the tasks that occur upon certain buttons are pressed on the controller. These are the prime candidates
to be handled by the scheduler. Upon controller input, one of the following can occur:

### Type A:

Certain flags change in ongoing functions that are called every loop to change the behaviour
1. Use mpu for locomotion stability
2. Invert the head of the robot
3. Change direction of spinning of a motor
4. Reset Lidar values / stop reading lidar
etc.

### Type B:

Certain actions are performed only when the inputs are present.
1. Dribble Claw toggle
2. Complete dribble sequence
3. Trigger transfer motor
4. Increase/decrease speed of throwing motor
5. Move turret mechanism (change yaw)
6. locomotion (upon joystick inputs)
7. Extend/Retract Defence mechanism
8. Change projectile entry angle for the throwing mechanism.

# Data Design Prototype:

```C++
#pragma once
#include "../API/Algos/Odometry/ThreeXYEncoder/ThreeXYEncoder.hpp"
#include "../API/Algos/PID/PID.h"
#include "../API/Drivers/Controller/Xbox/Xbox.h"
#include "../API/Drivers/Sensors/IMU/BNO/BNO.h"
#include "../API/Drivers/Sensors/LIDAR/Lidar.h"
#include <Arduino.h>
#include <cstdint>
///// Bot Config function prototypes
void botInit();

extern rudra::BNO bno;
extern PID pidLocoW;
extern PID pidLocoX;
extern PID pidLocoY;
extern USBHost myusb;
extern USBHIDParser hid;
extern XboxController Ps3;
extern BluetoothController mybt;

//////////////////////////////////////////////////////////////////////////
/// Data Structs for the robot tasks
//////////////////////////////////////////////////////////////////////////

/*----------------------------- Controller -----------------------------*/
#include "WProgram.h"

#define PS3_CONNECTED 0x01
#define PS3_RELAY_ON 0x03
#define PS3_PID_ON 0x07

struct ControllerState {
    int stickX, stickY, stickW;
    bool isRelayOn : 1;
    bool shouldDribble : 1;

    // Main controller Task
    void controllerTask();

    // Analog Hat functions
    void getLeftJoyVectors();
    void getRightJoyVectors();
    int getRightTrigger();
    int getLeftTrigger();

    // Digital Button functions
    void getButtonMaps();

private:
    void getJoyVectors(JoyEnum x, JoyEnum y, int& xVal, int& yVal);
};
/* == END == */

/*------------------------------ Odometry ------------------------------*/

struct OdometryState {
    float xSetpoint, ySetpoint, wSetpoint;
    float yaw;

    void odomTask();
};

/* == END == */

/*-------------------------------- PID ---------------------------------*/

struct PIDState {
    float locoW, locoY, locoX, turretOut;
#ifdef TUNE_PID
    void tunePid(PID& obj);
    void testPid(float setPoint, char dof);
#endif
};
/* == END == */

/*------------------------------- Motion -------------------------------*/
struct Motion {
    float valX, valY, valW; // Motion vectors
    float maSpeed, mbSpeed, mcSpeed, mdSpeed; // Motor speeds
    float maCap, mbCap, mcCap, mdCap; // Motor PWM caps
    bool invertAxis : 1; // Axis inversion flag
    bool mpUse : 1; // Yaw PID flag

    void drive(float argX, float argY, float argW);
    void getSpeed();
    void setMotion();
};
/* == END == */

/*------------------------------ Defence -------------------------------*/
struct DefenceState {
    enum extensionState { extend,
        stop,
        retract } state;

    void defenceTask();
};
/* == END == */

/*------------------------------ Dribble -------------------------------*/
struct Dribbler {
    unsigned long dribbleTimer, buttonDebounce;
    bool isBallInRange : 1;
    bool isClawOpen : 1;
    bool manual : 1;

    void dribbleTask(unsigned long);

private:
    void dribbleAuto(unsigned long);
    void dribbleManual();
};
/* == END == */

/*------------------------------- Turret -------------------------------*/
#define TURRET_ENC_PPR 286
#define TURRET_ENC_RES (TURRET_ENC_PPR / 360.0f) // 4x resolution

struct TurretYaw {
    long count;
    float angle;
    float setpoint;
    int velocity;
    bool useJoyForMovement : 1;

    bool turretTask();

private:
    void moveTurret();
};
/* == END == */

/*------------------------------ Thrower -------------------------------*/
struct Thrower {
    bool isThrowing : 1;
    int throwSpeed;

    bool throwTask();

private:
    void throwBall();
};
/* == END == */

/*------------------------------ Transfer ------------------------------*/
struct Transfer {
    bool transferUp : 1;
    bool transferDown : 1;
    bool sendBallToDribble : 1;

    bool transferPipeline();

private:
    void manualTransfer();
};
/* == END == */

/*------------------------------- Relay --------------------------------*/
struct RelayState {
    bool doAtRelayOnce : 1;

    void relayTask();
};
/* == END == */

extern ControllerState controller;
extern OdometryState odom;
extern Transfer transfer;
extern DefenceState defence;
extern RelayState relayState;
extern TurretYaw turretYaw;
extern Dribbler dribbler;
extern Motion motion;
extern PIDState pidState;
extern Thrower thrower;
```
