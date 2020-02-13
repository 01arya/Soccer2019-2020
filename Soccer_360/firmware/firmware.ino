
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


unsigned long t1_p=millis();//pixy_gioco

void setup()
{
  pinMode(35,OUTPUT);
  digitalWrite(35, HIGH);
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
    PhoenixIMU_handle(&_imu);
    Serial.println("IMU initialized...");
   }
   else
   {
     Serial.println("IMU insn't initialized...");

   }
   //init camera
  PhoenixCamera_init(&camera);
  Serial.println("Pixy is initialized...");
  digitalWrite(35, LOW);
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
    printIMU(&_imu);
    t0=t1;
  }
  return;
}

void Imu()
{
  //si orienta verso la imu
  if(imu_flag==true)
  {
    double output= get_output_pid(&_imu);
     t=-(output/180);
     y=0;
  }
  else
  {
    y=0;
    t=0;
  }
  PhoenixDrive_setSpeed(&drive,0,y,t);
}

double seguiNord()
{
  double output= get_output_pid(&_imu);
  t=-(output/180);
  PhoenixDrive_setSpeed(&drive,x,y,t);
  return t;
}
void triangolo(unsigned long t1_p)
{
  unsigned long t2_p=millis();
  if(t2_p<t1_p)
  {
    x=-1;
    y=-1;      
  }
  else
  {
    x=+1;
    y=-1;
  }
  double output= get_output_pid(&_imu);
  t=-(output/180);
  PhoenixDrive_setSpeed(&drive,x,y,t);
}

void segui_palla()
{
  //se vedo la palla

  if(PhoenixCamera_getBallStatus(&camera)==0)
  {
    //se la palla è davanti a noi
    if(PhoenixCamera_getBallY(&camera)>0)
    {
      x=PhoenixCamera_getBallX(&camera);
      y=PhoenixCamera_getBallY(&camera);
      t=seguiNord();
    } 
    //se la palla sta dietro a noi
    else
    {
      x=PhoenixCamera_getGoalX(&camera)-PhoenixCamera_getBallX(&camera);
      y=PhoenixCamera_getGoalY(&camera)-PhoenixCamera_getBallY(&camera);
      t=seguiNord(); 
    }
  }
  PhoenixDrive_setSpeed(&drive,x,y,t);
return;
}
void loop()
{

//testA_BFn();
readIMU(&_imu);
PhoenixIMU_handle(&_imu);
printIMU(&_imu);
PhoenixCamera_handle(&camera);

//seguiNord();
t1_p=millis();
triangolo(t1_p);
//segui_palla(t1_p);
PhoenixDrive_handle(&drive);


}
