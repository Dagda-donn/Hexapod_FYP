
// Axis system: X - front, Y - left, Z - top
// We assume that hexapod dimensions are simetrical in X and Y axis.

#include "Arduino.h"

// ****************************************************************************
// PIN NUMBERS
// In arduino Mega using 12 to 23 motors will disable PWM on pins 11 and 12.
#define LeftFrontCoxaPin 23
#define LeftFrontFemurPin 25
#define LeftFrontTibiaPin 27

#define RightFrontCoxaPin 22
#define RightFrontFemurPin 24
#define RightFrontTibiaPin 26

#define LeftMiddleCoxaPin 29
#define LeftMiddleFemurPin 31
#define LeftMiddleTibiaPin 35

#define RightMiddleCoxaPin 28
#define RightMiddleFemurPin 30
#define RightMiddleTibiaPin 34

#define LeftRearCoxaPin 37
#define LeftRearFemurPin 39
#define LeftRearTibiaPin 41

#define RightRearCoxaPin 26
#define RightRearFemurPin 38
#define RightRearTibiaPin 40

// ****************************************************************************
// SENSOR ADDRESSES (I2C)
#define UltrasonicSensorAddress 0x57  // Address for the ultrasonic sensor
#define HumiditySensorAddress 0x37    // Address for the humidity sensor
#define CompassSensorAddress 0x1E     // Address for the 3-axis compass sensor

// ****************************************************************************
// ANALOG SENSOR
#define SoilMoisturePin A0           // Analog pin for soil moisture sensor

// ****************************************************************************
// MIN & MAX ANGLES
// Angle expresed with 1 decimal : 90.0 = 900
#define LeftFemurMin -1000
#define LeftFemurMax 100
#define LeftTibiaMin -850
#define LeftTibiaMax 850

#define RightFemurMin -LeftFemurMax
#define RightFemurMax -LeftFemurMin
#define RightTibiaMin -LeftTibiaMax
#define RightibiaMax -LeftTibiaMin

#define LeftFrontCoxaMin -450
#define LeftFrontCoxaMax 450

#define RightFrontCoxaMin LeftFrontCoxaMin
#define RightFrontCoxaMax LeftFrontCoxaMax

#define LeftMiddleCoxaMin RightMiddleCoxaMin
#define LeftMiddleCoxaMax RightMiddleCoxaMax

#define RightMiddleCoxaMin LeftFrontCoxaMin
#define RightMiddleCoxaMax LeftFrontCoxaMax

#define LeftRearCoxaMin	RightRearCoxaMin
#define LeftRearCoxaMax	RightRearCoxaMax

#define RightRearCoxaMin LeftFrontCoxaMin
#define RightRearCoxaMax LeftFrontCoxaMax

// ****************************************************************************
// LEG DIMENSIONS in mm
#define CoxaLength 54
#define FemurLength 70
#define TibiaLength 155

// ****************************************************************************
// BODY DIMENSIONS in mm
#define CoxaOffsetX 78

#define CoxaOffsetY 55
#define MiddleCoxaOffsetY 78

// ****************************************************************************
// BODY DEFAULT ANGLE referred to axis X
// Angle expresed with 1 decimal : 90.0 = 900
#define LeftFrontCoxaDefaultAngle 450
#define RightFrontCoxaDefaultAngle -LeftFrontCoxaDefaultAngle
#define LeftMiddleCoxaDefaultAngle 900
#define RightMiddleCoxaDefaultAngle -900
#define LeftRearCoxaDefaultAngle 1350
#define RightRearCoxaDefaultAngle -LeftRearCoxaDefaultAngle

// ****************************************************************************
// LEG DEFFAULT POSITION referenced leg-axis
#define FootDistance 120
#define	FootHeight -110
#define DefaultTime 2000
#define Gap 30

// ****************************************************************************
// SERVO SPEED degrees per second (º/s)
// MG996r datasheet:
// 	- 0.17 s/60º 4.8v without load = 353º/s
// 	- 0.13 s/60º 6.8v without load = 461º/s 
#define MaxRealSpeed 370

// ****************************************************************************
// BODY MOVEMENT SPEED
// value [0,255] -> 255 = MaxSpeed
#define DefaultSpeed 180

// ****************************************************************************
// BODY ROTATION SPEED 
// balanceSpeed = _speed * BalanceSpeedProportion
#define BalanceSpeedProportion 0.85

// ****************************************************************************
// CONTROL
// ControlSerial    control hexapod by serial port
// ControlRos       control hexapod like a ROS node
#define ControlRos

// ControlMode
// 0 -> BasicControlMode (recommended) : 3 positions (high, middle, low)
// 1 -> AdvancedControlMode : You can control heigh, footDistance and Step distance
#define ControlMode 0

// BasicControlMode
#define BasicHighHeigh -150
#define BasicHighFootDistance FootDistance

#define BasicMiddleHeigh FootHeight
#define BasicMiddleFootDistance FootDistance

#define BasicLowHeigh -80
#define BasicLowFootDistance 150

// ****************************************************************************
// DEFAULT CONTROL STEPS
#define FootDistanceStep 10
#define SpeedStep 10
#define FloorHeightStep 10
