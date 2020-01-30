//mine
#pragma once
#include "phoenix_line_internals.h"
//#include "phoenix_globals.h"
#include <math.h>

/*
inizializza l (PhoenixLineSensor) azzerando i valori di
 * soglia, misura, misura_max, misura_min, detect_flag e 
 * calibra_flag
 * OCCHIO a NON azzerare le variabili x ed y !
 */
void PhoenixLineSensor_init(PhoenixLineSensor* d) 
{
  d->detect_flag=0;
  d->calibra_flag=0;
  d->misura=0;
  d->misura_min=65535;
  d->misura_max=0;
  d->soglia=0;
  pinMode(d->pin_reading,INPUT);
}

/*
 * Esegue la lettura del sensore, se la flag di calibrazione
 * (calibra_flag) e' attiva, allora aggiorna anche
 * misura_min e misura_max secondo queste condizioni:
 * misura < misura_min ? => misura_min = misura
 * misura > misura_max ? => misura_max = misura
 *
 *In oltre se la lettura e' maggiore della soglia
 * allora imposta ad 1 lo status (detect_flag)
 */
void PhoenixLineSensor_handle(PhoenixLineSensor* d) 
{
  d->misura=analogRead(d->pin_reading);
  if(d->calibra_flag==true)
  {
    if(d->misura>d->misura_max||d->misura==d->misura_max)
    {
      d->misura_max=d->misura;
    }
    if(d->misura<d->misura_min||d->misura==d->misura_min)
    {
      d->misura_min=d->misura;
    }
  }
  else
  {
    if(d->misura > d->soglia)
    {
      d->detect_flag = 1;
    }
    if(d->misura < d->soglia)
    {
      d->detect_flag = 0;
    }
  }
  
}

/*
 * imposta la variabile calibra_flag di l (PhoenixLineSensor)
 * pari a 1
 */
void PhoenixLineSensor_startCalib(PhoenixLineSensor* d)
{
  d->calibra_flag=1;
}

/*
 * imposta la variabile calibra_flag di l (PhoenixLineSensor)
 * pari a 0 
 * Poi imposta soglia secondo la formula:
 * soglia = (misura_max + misura_min) / 2
 * Poi azzera anche misura_min e misura_max
 */
void PhoenixLineSensor_stopCalib(PhoenixLineSensor* d)
{
  d->calibra_flag=0;
  d->soglia=(d->misura_max+d->misura_min)/2;
  d->misura_max=0;
  d->misura_min=65535;
}

/*
 * restituisce il valore di detect_flag
 */
uint8_t PhoenixLineSensor_getStatus(PhoenixLineSensor* d)
{
  return d->detect_flag;
}

/*
 * imposta la variabile  detect_flag di l (PhoenixLineSensor)
 * pari a 0
 */
void PhoenixLineSensor_reset(PhoenixLineSensor* d) 
{
  d->detect_flag=0;
  d->misura=0;
}
