/**
 * utils.h
 **/

#pragma once
#include <math.h>


inline double clamp(double v, double m) 
{
  if(v>m) return m;
  if(v<-m) return -m;
  return v;
}

inline int cconstraint(int v, int max, int min) 
{
  int interval=max-min;
  if(v<min) return v+interval;
  if(v>max) return v-interval;
  return v;
}

inline double degToRad(double deg) {return (deg*71)/4068;}
inline double radToDeg(double rad) {return (rad*4068)/71;}
inline double modulo(double x, double y) {return sqrt(x*x+y*y);}
