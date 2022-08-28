#ifndef COMMON_DEF_H
#define COMMON_DEF_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define LASER_POINTS 361
#define LASER_START_ANGLE (-M_PI_2)
#define LASER_ANGLE_RANGE (M_PI)

#define RANGE_INC 0.01

#define LASER_MAX_RANGE 8.0

#define DATA_PATH  "Data/"
#define MAX_READ 200
#define FILE_PATH_LIMIT 1000

#define FEATURE_TOL 0.1 //10cm
#define FEATURE_OUTLIER_TOL 2
// #define FEATURE_TOL 0.01 //10cm
// #define FEATURE_TOL 0.0025 //5cm

#define OUTLIER_TOL 2

#define USE_GNU_PLOT 1
#define GNU_UPDATE_FREQUENCY 50000 //If this number is large enough then it will only plot on the last cycle
// #define GNU_SPEED_UP 40

#define GNU_RAW "gnuPlot/rawData"

#define GNU_GPS "gnuPlot/gpsData"

#define GNU_FEAT "gnuPlot/featData"

#define GNU_COMB "gnuPlot/combData"

#define GNU_COMPASS "gnuPlot/compassData"


#define GNU_OCC_MATRIX "gnuPlot/laserOcc"
#define GNU_OCC_GRID "gnuPlot/occGrid"
#define GNU_FEAT_GRID "gnuPlot/featGrid"


#define WINDOW_SIZE_X 30.0 //Meters
#define WINDOW_SIZE_Y 30.0 //Meters

#define WINDOW_START_X -15.0 
#define WINDOW_START_Y -15.0 

#define WINDOW_RESOLTION 10 //How many blocks per meter (10 = 0.1m resolution)
#define WINDOW_ARRAY_X ((int)(WINDOW_SIZE_X*WINDOW_RESOLTION))
#define WINDOW_ARRAY_Y ((int)(WINDOW_SIZE_Y*WINDOW_RESOLTION))

#define COMP_GAIN 0.95
#define POS_GAIN 0.985

#define FEAT_GAIN 0.9
#define FEAT_HEAD_GAIN 0.1

#define OCC_THRESHOLD 0.6

#define EXPORT_CLOUD 1

typedef struct S_CALC_POS {
  double x;
  double y;
  double heading;
} CALC_POS;


typedef struct S_OCC_CELL {
  double hit;
  // double miss;
} OCC_CELL;


void clearComments(FILE *);

int isRel(void *, void *);

double computeSD(double *, double, int);

int computeRealAvg(double *, double *, int);

double normAngle(double, double);
double avgAngle(double *, int);

int calculateCell(OCC_CELL *, double, double);
int calculateCellHit(OCC_CELL * cell, double range);
int calculateCellHit(OCC_CELL * cell, double range, double offset);
int calculateCellMiss(OCC_CELL * cell, double range);
int calculateCellMiss(OCC_CELL * cell, double range, double offset);
int printFileInit();
int printPos(char *, double, double);
int printPos(char *, double, double, double);
int printPos(char *, double, double, double, double);

#endif
