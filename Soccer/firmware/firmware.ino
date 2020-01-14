
#include "phoenix_params.h"
#include "phoenix_joints.h"
#include "phoenix_drive.h"
#include "phoenix_globals.h"
#include "phoenix_imu.h"
#include "phoenix_pixy.h"
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

  if(!PhoenixImu_init(&imu)==0)
   {
    Serial.println("IMU initialized...");
    PhoenixImu_handle(&imu);
    PhoenixImu_setOffset(&imu,imu.heading_attuale);
    imu_flag=true;

   }
   else
   {
     Serial.println("IMU insn't initialized...");
     imu_flag=false;
   }

//pixy

  PhoenixCamera_init(&camera);
  Serial.println("Pixy initialized...");

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

void Imu()
{
   //si orienta verso la imu
  if(imu_flag==true)
  {
     t=-imu.output_pid/180;
     y=0;
  }
  else
  {
    y=-1;
    t=0;
  }
  PhoenixDrive_setSpeed(&drive,0,y,t);
}

void ball()
{
  if(imu_flag==true)
 {
   if(PhoenixCamera_getBallStatus(&camera)==1)
   {
    //traiettoria
    x=-sin(degToRad(imu.errore));
    y=1-cos(degToRad(imu.errore));
    t=-camera.output_pid/180;
    if(abs(x)<0.7)
    {
     y=1;
     /*if(camera.ball_y>175)
     {
       k=millis();
       if((millis()-k)>1000)
       {
          kiker();
       }*/
    }

    time=millis();
    x_prec=camera.ball_x;
    y_prec=camera.ball_y;
   }

  if (PhoenixCamera_getBallStatus(&camera)==0&&(millis()-time)<(2500))//se non la vedo da poco
  {
    //mi giro verso l'ultimo outpud_pid
    t=-camera.output_pid/180;
    //y
    //se la y<180 non ho la palletta
    //se la y>180 ho la palletta
    //più la palletta si allontana più y è piccolo
    if(y_prec<170)
    {
      y=0.75;
    }
  }

  else
  {
     if((millis()-time)>5000)//se non la vedo da tanto
     {
       t=0.5;
       y=0;
       x=0;
     }
     else
     {
       t=-camera.output_pid/180;
       y=-1;
     }
  }
 }
 if(imu_flag==false)//no bussola
 {
   if(PhoenixCamera_getBallStatus(&camera)==1)
   {
      t=(0.25)*(-camera.output_pid/180);
      x=camera.output_pid/180;
      y=0;
      time=millis();
      x_prec=camera.ball_x;
      y_prec=camera.ball_y;
   }
  else if((millis()-time)<(1000))//se non la vedo da poco
  {
    //mi giro verso l'ultimo outpud_pid
    x=camera.output_pid/180;
    //y
    //se la y<180 non ho la palletta
    //se la y>180 ho la palletta
    //più la palletta si allontana più y è piccolo
    if(y_prec<180)
    {
      y=1;
    }
  }
  else
  {
    if((millis()-time)>3000)//se non la vedo da tanto
    {
      t=0.5;
      y=0;
      x=0;
    }
   else
   {
    t=0;
    y=-1;
    x=0;
   }
  }
 }
 PhoenixDrive_setSpeed(&drive,x,y,t);
}
void ball_prova()
{
  if(PhoenixCamera_getBallStatus(&camera)==1)
   {
    //traiettoria
    x=-sin(degToRad(imu.errore));
    y=1-cos(degToRad(imu.errore));
    t=-camera.output_pid/180;
    if(abs(x)<0.7)
    {
     y=1;
    }

    time=millis();
    x_prec=camera.ball_x;
    y_prec=camera.ball_y;
   }

  PhoenixDrive_setSpeed(&drive,x,y,t);

  Serial.print("x= ");
  Serial.print(x);
  Serial.print("\t");

  Serial.print("y= ");
  Serial.print(y);
  Serial.print("\t");

  Serial.print("t= ");
  Serial.print(t);
  Serial.println();
}
void loop()
{

//testA_BFn();

PhoenixImu_handle(&imu);
//PhoenixCamera_handle(&camera);

//testDriveFn();
PhoenixImu_print(&imu);
//Serial.println(imu.errore);
Imu();

//ball_prova();


PhoenixDrive_handle(&drive);
//PhoenixCamera_print(&camera);

}
