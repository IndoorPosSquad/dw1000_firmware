#ifndef __SOLVE_H
#define __SOLVE_H

typedef struct XYZ {
  float x;
  float y;
  float z;
} xyz;


void solve_2d(float reciever[2][2], float pseudolites[2][2], float pranges1, float pranges2);
xyz solve_3d(xyz pseudolites[3], float pranges[3]);

#endif
