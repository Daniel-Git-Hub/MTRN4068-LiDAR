#ifndef FEATURE_H
#define FEATURE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common_def.h"


typedef struct S_FEATURE {
  double x;
  double y;
} FEATURE;

typedef struct S_FEATURE_DISTANCE {
  double range;
  double angle;
  int idx;
} FEATURE_DISTANCE;


int featInit();
int featClose();
int featPrintAll();

FEATURE * featGet(int);

int featGetClosest(double, double);
int featGetClosest(CALC_POS *);
int featCount();

double featGetDistance(int, double, double);
double featGetClosestDistance(double x, double y);
#endif  