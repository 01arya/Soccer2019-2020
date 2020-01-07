
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "phoenix_joints.h"
#include "phoenix_drive.h"
#include "phoenix_globals.h"
#include "phoenix_imu.h"
#include "utils.h"


#define pin_k 7
struct Timer* test1;
bool cal=false;
double x=0;
double y=0;
double t=0;
double x_prec=0;
double y_prec=0;
double time;//tempo in cui è stata vista l'ultima volta la palla
bool imu_flag=false;//controlla se la imu è inizializzata
int k;

void setup() 
{
  Serial.begin(9600);
  Serial.println("Serial initialized...");

  for(int i=0;i<NUM_JOINTS;++i) 
  {
    PhoenixJoint_init(&joints[i]);
  }
  Serial.println("Joints initialized...");
  PhoenixDrive_init(&drive, joints);
  Serial.println("Drive initialized...");

  //imu init
if(!initIMU()==0)
   {
    imu_flag=true;
    PhoenixIMU_handle(&IMU);
    Serial.println("IMU initialized...");
   }
   else
   {
     Serial.println("IMU insn't initialized...");
    
   }
}

volatile uint16_t idle_time=0;
volatile uint8_t test_joint_fn_state=0;
volatile uint8_t test_joint_fn_joint_idx=0;



void testJointsFn() 
{
  switch(test_joint_fn_state) 
  {
  case 0:
    PhoenixJoint_setSpeed(&joints[test_joint_fn_joint_idx%3], 255);
    PhoenixJoint_handle(&joints[test_joint_fn_joint_idx%3]);
    break;
  case 1:
    PhoenixJoint_setSpeed(&joints[test_joint_fn_joint_idx%3], -255);
    PhoenixJoint_handle(&joints[test_joint_fn_joint_idx%3]);
    break;
  case 3:
    PhoenixJoint_setSpeed(&joints[test_joint_fn_joint_idx%3], 0);
    PhoenixJoint_handle(&joints[test_joint_fn_joint_idx%3]);
    test_joint_fn_joint_idx++;
    break;
  }
  test_joint_fn_state++;
  if(test_joint_fn_state>3)
    test_joint_fn_state=0;
}

void testA_BFn()
{

  PhoenixJoint_setSpeed(&joints[0],255);
  PhoenixJoint_handle(&joints[0]);
  delay(1000);
  PhoenixJoint_setSpeed(&joints[0],-255);
  PhoenixJoint_handle(&joints[0]);
  delay(1000);
  PhoenixJoint_setSpeed(&joints[0],0);
  PhoenixJoint_handle(&joints[0]);
  delay(1000);

  PhoenixJoint_setSpeed(&joints[1],255);
  PhoenixJoint_handle(&joints[1]);
  delay(1000);
  PhoenixJoint_setSpeed(&joints[1],-255);
  PhoenixJoint_handle(&joints[1]);
  delay(1000);
  PhoenixJoint_setSpeed(&joints[1],0);
  PhoenixJoint_handle(&joints[1]);
  delay(1000);

  PhoenixJoint_setSpeed(&joints[2],255);
  PhoenixJoint_handle(&joints[2]);
  delay(1000);
  PhoenixJoint_setSpeed(&joints[2],-255);
  PhoenixJoint_handle(&joints[2]);
  delay(1000);
  PhoenixJoint_setSpeed(&joints[2],0);
  PhoenixJoint_handle(&joints[2]);
  delay(1000);

}
void testDriveFn()
{
  PhoenixDrive_setSpeed(&drive, 1, 0, 0);
  //(motore,x,y,rotazione)
  PhoenixDrive_handle(&drive);
}
  unsigned long t0=0;
  unsigned long t1=0;

void print_imu()
{
  t1=millis();
  if(t1-t0>10)
  {
    printIMU(&IMU);
    t0=t1;
  }
  return;
}

void Imu()
{
  //si orienta verso la imu
  if(imu_flag==true)
  {
    double output= get_output_pid(&IMU);
     t=-(output/180);
     y=0;
  }
  else
  {
    y=-1;
    t=0;
  }
  PhoenixDrive_setSpeed(&drive,0,y,t);
}


void loop() 
{

//testA_BFn();
readIMU(&IMU);
//Imu();
print_imu();

Imu();

//PhoenixDrive_setSpeed(&drive,0,0.0,255);

PhoenixDrive_handle(&drive);

  
}