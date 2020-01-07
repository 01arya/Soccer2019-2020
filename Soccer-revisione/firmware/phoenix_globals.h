/**
 * phoenix_globals.h
 **/

#pragma once

#include <Arduino.h>
#include "phoenix_joints.h"
#include "phoenix_drive.h"
#include "phoenix_imu.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

/*
se provi a compilare dice che PhoenixBNO non è un tipo
*/

//extern unsigned long time_imu0;
//extern unsigned long time_imu1;

extern PhoenixJoint joints[NUM_JOINTS];
extern PhoenixDrive drive;
extern Adafruit_BNO055 bno;
extern PhoenixBNO IMU;
/*
#include "phoenix_line.h"
#include "phoenix_line_internals.h"
*/


/*
extern PhoenixLineSensor line_sensors[NUM_LINE_SENSORS];
extern PhoenixLineHandler line_handler;
*/

