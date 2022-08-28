#ifndef POINT_H
#define POINT_H

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <string.h>
#include "common_def.h"
#include "feature.h"


typedef struct S_POINT_CLOUD {
  FEATURE * features;
  int featureCount;
  int maxFeatureCount;
  // double avgX;
  // double avgY;
  double worldX;
  double worldY;
} POINT_CLOUD;

typedef struct S_BRUTE_RETURN {
  int status;
  int featureIdx;
  double angle;
} BRUTE_RETURN;

int pointClose();

int pointAdd(FEATURE *);
BRUTE_RETURN pointBruteIter(CALC_POS *, CALC_POS *, int);
int pointBrute(CALC_POS *, CALC_POS *, int);
int pointCloseGnu();
int pointPrintCloud();
#endif  