/**
 * phoenix_globals.cpp
 **/


#include "phoenix_globals.h"



/*
*                         *
**********YATILI***********
*                         *
*/
PhoenixJoint joints[NUM_JOINTS] =
{
  //senso orario
  //dietro: joint 0
  //destra: joint 1
  //sinistra: joint 2
  {
  // Joint 0
  pin_dira : 23,
  pin_dirb : 28,
  pin_pwm : 3,
  direzione : 0,
  velocita : 0
  },
  {
  // Joint 1
  pin_dira : 22,
  pin_dirb : 27,
  pin_pwm : 2,
  direzione : 0,
  velocita : 0
  },
  {
  // Joint 2
  pin_dira : 24,
  pin_dirb : 29,
  pin_pwm : 4,
  direzione : 0,
  velocita : 0
  }
};
PhoenixDrive drive;
/*

PhoenixLineSensor line_sensors[NUM_LINE_SENSORS] =
{
  {// Sensore 0
  x : 39.00,
  y : 67.5499,
  soglia : 0,
  misura : 0,
  misura_min : 65535,
  misura_max : 0,
  detect_flag : 0,
  calibra_flag : 0,
  pin_reading: A0
  },
  {// Sensore 1
  x : 84.051,
  y : -22.2011,
  soglia : 0,
  misura : 0,
  misura_min : 65535,
  misura_max : 0,
  detect_flag : 0,
  calibra_flag : 0,
  pin_reading: A1
  },
  {// Sensore 2
  x : 61.6838,
  y : -61.2470,
  soglia : 0,
  misura : 0,
  misura_min : 65535,
  misura_max : 0,
  detect_flag : 0,
  calibra_flag : 0,
  pin_reading: A2
  },
  {// Sensore 3
  x : -61.2579,
  y : -61.6668,
  soglia : 0,
  misura : 0,
  misura_min : 65535,
  misura_max : 0,
  detect_flag : 0,
  calibra_flag : 0,
  pin_reading: A3
  },
  {// Sensore 4
  x : -83.8843,
  y : -22.7760,
  soglia : 0,
  misura : 0,
  misura_min : 65535,
  misura_max : 0,
  detect_flag : 0,
  calibra_flag : 0,
  pin_reading: A4
  },
  {// Sensore 5
  x : -39.00,
  y : 67.5499,
  soglia : 0,
  misura : 0,
  misura_min : 65535,
  misura_max : 0,
  detect_flag : 0,
  calibra_flag : 0,
  pin_reading: A5
  }
};
PhoenixLineHandler line_handler;
*/

Adafruit_BNO055 bno=Adafruit_BNO055();
PhoenixImu _imu =
{
  heading_attuale:0,
  heading_target:0,
  heading_offset:0,
  errore:0,
  output_pid:0,
  max_output:180,
  ki:0.0002,
  kp:3.5,
  kd:0.0010,
  errore_prec:0,
  dt:0.001,
  idt:1000,
  sum_i:0,
  max_i:180,
  time_imu0:0,
  time_imu1:0,
};

/*
  ki:00,
  kp:3.5,
  kd:0.0050,

  ki:0.0002,
  kp:3.5,
  kd:0.0010,
*/

//PhoenixBNO IMU;

/*

PhoenixCamera camera=
{
  ball_detection:0,
  ball_x:0,
  ball_y:0,
  
  ball_x_t=0,
  ball_y_t=0,
 
  ball_w:0,
  ball_h:0,
  ki:00,
  kp:0,
  kd:0.000,
  output_pid:0,
  max_output:180,
  errore:0,
  errore_prec:0,
  dt:0.00,
  idt:00.0,
  sum_i:0,
  max_i:180,
};

 */