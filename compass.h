#ifndef COMPASS_H
#define COMPASS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


typedef struct S_COMP {
  unsigned long second;
  unsigned int micro;
  double heading;
} COMP;



int compInit();
int compClose();

COMP * compGetCurrent();

COMP * compAdvance();

#endif