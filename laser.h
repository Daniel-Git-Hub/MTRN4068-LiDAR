#ifndef LASER_H
#define LASER_H

#include "common_def.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


typedef struct S_LASER {
  unsigned long second;
  unsigned int micro;
  double range[LASER_POINTS];
  char intensity[LASER_POINTS];
} LASER;



int laserInit();
int laserClose();

LASER * laserGetCurrent();

LASER * laserAdvance();
int laserPrintIndex(int i);


#endif  