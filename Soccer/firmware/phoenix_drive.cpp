//mine

#include "phoenix_joints.h"
#include "phoenix_drive.h"
#include "phoenix_params.h"


/**
 * matrice cinematica robot a 3 assi
 * da velocita motori => forze su robot

static double kmatrix[NUM_JOINTS][NUM_JOINTS] =
{
  //matirce STEORA
  
  {1.0, -0.5, -0.5},
  {0.0, 0.86, -0.86},
  {1.0, 1.0, 1.0}

  //matrice YATILI
  {1.0, -0.5, -0.5},
  {0.0, 0.86, -0.86},
  {1.0, 1.0, 1.0}

};
*/

/**
 * matrice inversa cinematica robot a 3 assi
 * da forze su robot => velocita motori
 **/
static double ikmatrix[NUM_JOINTS][NUM_JOINTS] =
{
    //matrice inversa STEORA
    /*
    {0.6,	0, 0.33},
    {-0.33, 0.58, 0.33},
    {-0.33, -0.58, 0.33}
    */
    //matrice inversa YATILI
    {0.6,	0, 0.33},
    {-0.33, 0.58, 0.33},
    {-0.33, -0.58, 0.33}
};

/**
 * inizializza d (PhoenixDrive) azzerando le velocita desiderate
 * e regolate.
 * In oltre  imposta d->joints = joint_array
 **/
void PhoenixDrive_init(PhoenixDrive* d, PhoenixJoint* joint_array)
{
  d->vel_x=0;
  d->vel_y=0;
  d->rot=0;
  d->joints=joint_array;
}

/**
 * imposta le velocita desiderate di d, in funzione dei parametri
 * x, y e r
 */
void PhoenixDrive_setSpeed(PhoenixDrive* d, double x, double y, double r)
{
  d->vel_x=x;
  d->vel_y=y;
  d->rot=r;
}

/**
 * Calcola ed assegna le velocita per
 * ogni Joint presente in d->joints
 **/
void PhoenixDrive_handle(PhoenixDrive* d)
{
  double comp_vec[3] = {d->vel_x,d->vel_y,d->rot};


  for(int r=0;r<NUM_JOINTS;r++)
  {
    double speed=0;
    for(int j=0;j<NUM_JOINTS;j++)
    {
      speed+=ikmatrix[r][j]*comp_vec[j];
      //calcola la velocità di joints->speed la matrice con la matrice in alto e il vettore {rot, x, y}
    }
    PhoenixJoint_setSpeed(&d->joints[r],speed*255);

    //ogni elemento dell'array è riferito ad un motore
    //ogni elemento ha un parametro speed
  }
  for(int a=0;a<NUM_JOINTS;a++)
  {
     PhoenixJoint_handle(&d->joints[a]);
  }

}

/**
 * Azzera i valori di velocita desiderate in d (PhoenixDrive)
 * e rilancia PhoenixDrive_handle
 **/
void PhoenixDrive_reset(PhoenixDrive* d)
{
  d->vel_x=0;
  d->vel_y=0;
  d->rot=0;
  PhoenixDrive_handle(d);
}
