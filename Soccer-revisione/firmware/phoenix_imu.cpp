
#include <Arduino.h>
#include "phoenix_imu.h"
#include "phoenix_globals.h"
#include "utils.h"
//inizializzazione della imu
uint8_t initIMU()
{
  bno.begin(bno.OPERATION_MODE_IMUPLUS);
  bno.setExtCrystalUse(true);
  return 0;
}

void readIMU(PhoenixImu* b)
{
    b->time_imu1=millis();
    if((b->time_imu1-b->time_imu0)>5)
    {
        imu::Vector<3>euler=bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        if(euler.x()!=b->heading_attuale)
        {
          b->heading_attuale=cconstraint(euler.x(),180,-180);
        }
        b->time_imu0=b->time_imu1;
    }    
}

void PhoenixIMU_handle(PhoenixImu* b) 
{
    b->time_imu1=millis();
    if((b->time_imu1-b->time_imu0)>5)
    readIMU(b);
  b->errore=b->heading_target-(b->heading_attuale-b->heading_offset);
  //m->heading_attuale = cconstraint(m->errore,180,-180);
  b->errore = cconstraint(b->errore,180,-180);
  //constain serve a limitare i gradi fino a 180 e -180 pittosto che lasciare i gradi a 360
  
  //PID
  //ERRORE PROPORZIONALE
  //quanto è grande l'errore
  double e_p=b->errore*b->kp;


  //ERRORE DERIVATIVO
  // calcola l'errore nel tempo
  double e_d=((b->errore-b->errore_prec)*b->idt)*b->kd;

  //ERRORE INTERGRALE
  //errori che aumentano nel tempo
  b->sum_i+=b->ki*b->errore*b->dt;
  /*
  divergenza dell'integrale:rischiamo che sum_i diventi enorme
  gli impostiamo un limite
  un valore massimo e un valore minimo
  per controllare questo faccio clamp
  */
  b->sum_i=clamp(b->sum_i,b->max_i);
 
  b->output_pid=e_p+e_d+b->sum_i;
 // dobbiamo limitare l'output
  b->output_pid=clamp(b->output_pid,b->max_output);

  b->errore_prec=b->errore;
 
   return; 
}

void printIMU(PhoenixImu *b)
{

    Serial.print("imu_x: ");
    Serial.print(b->heading_attuale);
    Serial.print("\t");

    
    Serial.print("output_pid: ");
    Serial.print(b->output_pid);
    Serial.print("\t");

    Serial.print("imu_offset: ");
    Serial.print(b->heading_offset);
    Serial.print("\t");

    Serial.print("imu_errore: ");
    Serial.print(b->errore);
    Serial.println("\t");

    return;
}


double get_output_pid(PhoenixImu* b)
{
    return b->output_pid;
}