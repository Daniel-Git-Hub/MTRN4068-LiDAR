#ifndef VEL_H
#define VEL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


typedef struct S_VEL {
  unsigned long second;
  unsigned int micro;
  double turnRate;
  double vel;
} VEL;



int velInit();
int velClose();

VEL * velGetCurrent();

VEL * velAdvance();

double velGetDt();


#endif