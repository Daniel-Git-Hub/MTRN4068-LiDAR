#ifndef POS_H
#define POS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


typedef struct S_POS {
  unsigned long second;
  unsigned int micro;
  double x;
  double y;
} POS;



int posInit();
int posClose();

POS * posGetCurrent();

POS * posAdvance();

#endif    