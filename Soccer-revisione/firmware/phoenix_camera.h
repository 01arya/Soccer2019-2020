/**
 * mine
 * e' un interfaccia personalizzata per PixyCam
 * tramite la quale e' possibile determinare la posizione di palla e
 * porte (alleata e nemica).
*/

/**
 * phoenix_camera.h
 **/

#pragma once
#include <Arduino.h>
#define BALL_SIG 1
#define BALL_RELIABLE_AGE 5

typedef struct {
  int8_t ball_detection;
  uint16_t ball_x;
  uint16_t ball_y;
  uint16_t ball_w;
  uint16_t ball_h;
  //per il pid
  double ki;//ERRORE INTERGRALE
  double kp;//ERRORE PROPORZIONALE
  double kd;//ERRORE DERIVATIVO
  double output_pid;
  double max_output;
  double errore;
  double errore_prec;
  double dt;
  double idt;
  double sum_i;
  int max_i;
}PhoenixCamera;


/**
 * Inizializza p (PhoenixCamera*) azzerando i valori
 * ball_detection, ball_x, ball_y, ball_w e ball_h
 * In oltre è necessario inizializzare 
 * l'oggetto pixy (Pixy) tramite il metodo pixy.init()
 **/
void PhoenixCamera_init(PhoenixCamera* p);

/**
 * Interroga il modulo pixy, richiedendo il numero di blocchi
 * visti.
 * I passi necessari sono questi:
 * 1) Imposta ball_detection pari a 0
 * 2) Richiede numero di blocchi visti (pixy.getBlocks())
 * 3) Se almeno un blocco è stato rilevato, bisogna controllare
 *    se uno dei blocchi ha la stessa "signature" della palla
 *    (salvata in BALL_SIG) allora bisogna incrementare
 *    la variabile ball_detection;
 *    La variabile in questione deve essere limitata all'intervallo
 *    (0, BALL_RELIABLE_CTR)
 * 4) Una volta trovata la palla, scaricare i seguenti valori
 *    dalla pixy:
 *    - x (coordinata x del blocco)
 *    - y (coordinata y del blocco)
 *    - w (larghezza del blocco [width])
 *    - h (altezza x del blocco [height])
 * 5) Se non viene rilevato nessun blocco oppure non viene rilevata
 *    la palla, decrementare la variabile ball_detection
 **/
void PhoenixCamera_handle(PhoenixCamera* p);

/**
 * Restituisce il valore ball_detection
 **/
uint8_t PhoenixCamera_getBallStatus(PhoenixCamera* p);

/**
 * Restituisce il valore ball_x
 **/
uint16_t PhoenixCamera_getBallX(PhoenixCamera* p);

/**
 * Restituisce il valore ball_y
 **/
uint16_t PhoenixCamera_getBallY(PhoenixCamera* p);

/**
 * Restituisce il valore ball_w
 **/
uint16_t PhoenixCamera_getBallW(PhoenixCamera* p);

/**
 * Restituisce il valore ball_h
 **/
uint16_t PhoenixCamera_getBallH(PhoenixCamera* p);

void PhoenixCamera_print(PhoenixCamera*p);