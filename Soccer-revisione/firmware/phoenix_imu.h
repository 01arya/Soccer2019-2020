#pragma once

#include <Arduino.h>
#include "phoenix_globals.h"
#include "utils.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

/*
se provi a compilare dice che PhoenixBNO non è un tipo
*/

typedef struct
{
  double heading_attuale;//attuale
  double heading_target;//obbiettivo
  double heading_offset;//differenza rispetto ad un valore di riferimento
  double errore;//errore attuale
  double output_pid;//il valore di correzione che mi restitusci il pid
  double max_output;//massimo valore di outpid
  //per il pid
  double ki;//ERRORE INTERGRALE
  double kp;//ERRORE PROPORZIONALE
  double kd;//ERRORE DERIVATIVO

  double errore_prec;//errore precedente
  double dt;//delta tempo
  double idt;//inverso delta tempo

  double sum_i;//sommatoria integrale
  int max_i;//massimo valore di sum_i

 unsigned long time_imu0;
 unsigned long time_imu1;
}PhoenixBNO;

uint8_t initIMU();
void readIMU(PhoenixBNO* b);
void PhoenixIMU_handle(PhoenixBNO* b); 
void printIMU(PhoenixBNO* b);
double get_output_pid(PhoenixBNO* b);