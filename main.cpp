// #include "ros/ros.h"
// #include "std_msgs/String.h"

// #include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "common_def.h"
#include "compass.h"
#include "feature.h"
#include "laser.h"
#include "pos.h"
#include "vel.h"
#include "point.h"
#include "gnuplot_i.h"
#include <unistd.h>



int laserCount = 0;
double testSum = 0;

CALC_POS calcPos;
CALC_POS rawPos;
CALC_POS featPos;

char occopuncyGrid[WINDOW_ARRAY_Y][WINDOW_ARRAY_X];

OCC_CELL complexOccopuncyGrid[WINDOW_ARRAY_Y][WINDOW_ARRAY_X];

unsigned long initSec;

int main(int argc, char **argv)
{
  printf("Start Cmd Define\n");
  char * plotCmd = 
"set size 1.0, 1.0\n"
"set origin 0.0, 0.0\n"
"set multiplot layout 2,2 rowsfirst margins 0.1,0.9,0.1,0.9 spacing 0.1\n"
"set xlabel 'Time (s)'\n"
"set ylabel 'x (m)'\n"
// "set title 'Time vs X'\n"
"plot '" GNU_RAW "' volatile using 't':'x' title 'Raw' with lines, "
"'" GNU_GPS "' volatile using 't':'x' title 'GPS' with lines, "
"'" GNU_FEAT "' volatile using 't':'x' title 'LiDAR' with lines, "
"'" GNU_COMB "' volatile using 't':'x' title 'Combined' with lines\n"
"set ylabel 'y (m)'\n"
// "set title 'Time vs Y'\n"
"plot '" GNU_RAW "' volatile using 't':'y' title 'Raw' with lines, "
"'" GNU_GPS "' volatile using 't':'y' title 'GPS' with lines, "
"'" GNU_FEAT "' volatile using 't':'y' title 'LiDAR' with lines, "
"'" GNU_COMB "' volatile using 't':'y' title 'Combined' with lines\n"
"set ylabel 'heading (rad)'\n"
// "set title 'Time vs Heading'\n"
"plot '" GNU_RAW "' volatile using 't':'h' title 'Raw' with lines, "
"'" GNU_COMPASS "' volatile using 't':'h' title 'Compass' with lines, "
"'" GNU_FEAT "' volatile using 't':'h' title 'LiDAR' with lines, "
"'" GNU_COMB "' volatile using 't':'h' title 'Combined' with lines\n"
"set ylabel 'y (m)'\n"
"set xlabel 'x (m)'\n"
// "set title 'X vs Y'\n"
"plot '" GNU_RAW "' volatile using 'x':'y' title 'Raw' with lines, "
"'" GNU_GPS "' volatile using 'x':'y' title 'GPS' with lines, "
"'" GNU_FEAT "' volatile using 'x':'y' title 'LiDAR' with lines, "
"'" GNU_COMB "' volatile using 'x':'y' title 'Combined' with lines\n"
"unset multiplot\n"
  ;

  char * occCmd = 
"set ylabel 'y (m)'\n"
"set xlabel 'x (m)'\n"
"set xrange [-15:15]\n"
"set yrange [-15:15]\n"
"set pointsize 5\n"
// "set title 'Occupancy Grid'\n"
"plot '" GNU_OCC_MATRIX "' matrix using ($1/10 - 15):($2/10 - 15):3 with image, "
"'" GNU_COMB "' volatile using 'x':'y' title 'Robot Path' with lines, "
// "'" GNU_OCC_GRID "' volatile using 'x':'y' title 'GPS' with points pt '⊙', "
"'" GNU_FEAT_GRID "' volatile using 'x':'y' title 'Features' with points pt '⊙'\n"
;

  printf("Init start\n");



#ifdef USE_GNU_PLOT
  printFileInit();
#if USE_GNU_PLOT == 1
  gnuplot_ctrl * gnuPlot, * occPlot;
  gnuPlot = gnuplot_init();
  // gnuplot_cmd(gnuPlot, plotCmd);
  
  occPlot = gnuplot_init();
  // gnuplot_cmd(occPlot, occCmd);
#endif
#endif

  compInit();

  featInit();

  posInit();

  velInit();

  laserInit();

  for(int i = 0; i < WINDOW_ARRAY_Y; i++){
    for(int ii = 0; ii < WINDOW_ARRAY_X; ii++){
      complexOccopuncyGrid[i][ii].hit = 0.5;
    }

  }

  memset(occopuncyGrid, 0, (WINDOW_ARRAY_Y)*(WINDOW_ARRAY_X));

  COMP * currentComp = compGetCurrent();
  POS * currentPos = posGetCurrent();
  LASER * currentLaser = laserGetCurrent();

  COMP * keepComp = compGetCurrent();
  POS * keepPos = posGetCurrent();
  LASER * keepLaser = laserGetCurrent();

  calcPos.x = currentPos->x;
  calcPos.y = currentPos->y;
  calcPos.heading = currentComp->heading;

  rawPos = calcPos;
  featPos = calcPos;
  
  double normHeading = rawPos.heading;

  VEL * currentVel = velGetCurrent();
  
  initSec = currentVel->second;
  
  printf("Init finish\n");

  int loopCount = 0;
  while(1){
    loopCount++;
    double dt = velGetDt();
    double time = currentVel->second - initSec;
    time += 0.000001*currentVel->micro;


    rawPos.heading += dt*currentVel->turnRate;
    calcPos.heading += dt*currentVel->turnRate;

    if(currentComp && isRel(currentVel, currentComp)){
      normHeading = normAngle(calcPos.heading, keepComp->heading);
      calcPos.heading = COMP_GAIN*calcPos.heading + (1-COMP_GAIN)*normHeading;

#ifdef USE_GNU_PLOT
      printPos(GNU_COMPASS, time, normHeading);
#endif

      if(currentComp){
        keepComp = currentComp;
      }
      currentComp = compAdvance();
    }
    // double normHeading = normAngle(calcPos.heading, keepComp->heading);
    // calcPos.heading = COMP_GAIN*calcPos.heading + (1-COMP_GAIN)*normHeading;
    

    calcPos.x += dt*(currentVel->vel)*cos(calcPos.heading);
    calcPos.y += dt*(currentVel->vel)*sin(calcPos.heading);

    rawPos.x += dt*(currentVel->vel)*cos(rawPos.heading);
    rawPos.y += dt*(currentVel->vel)*sin(rawPos.heading);

#ifdef USE_GNU_PLOT
    printPos(GNU_RAW, time, rawPos.x, rawPos.y, rawPos.heading);
#endif


    if(currentPos && isRel(currentVel, currentPos)){

      calcPos.x = POS_GAIN*calcPos.x + (1-POS_GAIN)*currentPos->x;
      calcPos.y = POS_GAIN*calcPos.y + (1-POS_GAIN)*currentPos->y;

#ifdef USE_GNU_PLOT
      printPos(GNU_GPS, time, currentPos->x, currentPos->y);
#endif

      if(currentPos){
        keepPos = currentPos;
      }
      currentPos = posAdvance();
      
    }
 

    if(currentLaser && isRel(currentVel, currentLaser)){
      
      for(int x = 0; x < LASER_POINTS; x++){

        double laserAngle = LASER_START_ANGLE + x*LASER_ANGLE_RANGE/(LASER_POINTS-1);

        if(currentLaser->intensity[x]){
          double laserAngle = LASER_START_ANGLE + x*LASER_ANGLE_RANGE/(LASER_POINTS-1);

          FEATURE newPoint;
          newPoint.x = calcPos.x + currentLaser->range[x]*cos(laserAngle+calcPos.heading);
          newPoint.y = calcPos.y + currentLaser->range[x]*sin(laserAngle+calcPos.heading);

          pointAdd(&newPoint);

        }
      }


#ifdef USE_GNU_PLOT
      if((laserCount) == 200){
        // printf("Lasercount %lf %lf, %lf, %lf\n",time, featPos.x, featPos.y, featPos.heading);
        FILE * laserFile = fopen("gnuPlot/laserRobotPos.csv", "w");
        fprintf(laserFile, "idx, time, x, y, heading\n");
        fprintf(laserFile, "%d, %lf, %lf, %lf, %lf\n", 0, time, calcPos.x, calcPos.y, calcPos.heading);
        fclose(laserFile);
        pointPrintCloud();
      }
#endif

    BRUTE_RETURN bruteResult = pointBruteIter(&calcPos, &featPos, laserCount == 200);

    if(!bruteResult.status){

      double tempX = calcPos.x + featPos.x;

      double tempY = calcPos.y + featPos.y; 
      featPos.heading = normAngle(calcPos.heading, calcPos.heading + bruteResult.angle);

      featPos.x = tempX;
      featPos.y = tempY;

      FEATURE * feat = featGet(bruteResult.featureIdx);
      tempX = featPos.x - feat->x;
      tempY = featPos.y - feat->y;

      featPos.x = tempX*cos(bruteResult.angle) - tempY*sin(bruteResult.angle);
      featPos.y = tempX*sin(bruteResult.angle) + tempY*cos(bruteResult.angle);

      featPos.x = featPos.x + feat->x;
      featPos.y = featPos.y + feat->y;

      
#ifdef USE_GNU_PLOT
      printPos(GNU_FEAT, time, featPos.x, featPos.y, featPos.heading);
#endif

      if(abs(featPos.x - calcPos.x) <= FEATURE_OUTLIER_TOL && abs(featPos.y - calcPos.y) <= FEATURE_OUTLIER_TOL){
        calcPos.heading = FEAT_HEAD_GAIN*calcPos.heading + (1-FEAT_HEAD_GAIN)*(featPos.heading);
        calcPos.x = FEAT_GAIN*calcPos.x + (1-FEAT_GAIN)*featPos.x;
        calcPos.y = FEAT_GAIN*calcPos.y + (1-FEAT_GAIN)*featPos.y;
      }

    }

      

      //compute occupancy grid
      for(int a = 0; a < LASER_POINTS; a++){

          //Bresenham's line algorithm, code is a modifed version of low level version described in https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
          double laserAngle = LASER_START_ANGLE + a*LASER_ANGLE_RANGE/(LASER_POINTS-1);
          double x0D = WINDOW_RESOLTION*(calcPos.x - WINDOW_START_X);
          double y0D = WINDOW_RESOLTION*(calcPos.y - WINDOW_START_Y);
          double x1D = WINDOW_RESOLTION*(calcPos.x + currentLaser->range[a]*cos(laserAngle+calcPos.heading) - WINDOW_START_X);
          double y1D = WINDOW_RESOLTION*(calcPos.y + currentLaser->range[a]*sin(laserAngle+calcPos.heading) - WINDOW_START_Y);
          double aLine = y1D - y0D;
          double bLine = x0D - x1D;
          double abDist = sqrt(pow(aLine,2) + pow(bLine,2));
          double cLine = -(aLine*x0D + bLine*y0D);

          int x0 = floor(x0D);
          int y0 = floor(y0D);
          int x1 = floor(x1D);
          int y1 = floor(y1D);
          int dx = abs(x1 - x0);
          int sx = x0 < x1 ? 1 : -1;
          int dy = -abs(y1 - y0);
          int sy = y0 < y1 ? 1 : -1;
          int error = dx + dy;

          int hitState = 0;
          if(currentLaser->range[a] < LASER_MAX_RANGE){
            hitState = 1;
          }

          while(1){

            if(x0 < 0 || x0 >= ((int)WINDOW_SIZE_X*WINDOW_RESOLTION) || y0 < 0 || y0 >= ((int)WINDOW_SIZE_Y*WINDOW_RESOLTION)){
              break;
            }

            double xCurrent = ((x0+0.5)/WINDOW_RESOLTION + WINDOW_START_X);
            double yCurrent = ((y0+0.5)/WINDOW_RESOLTION + WINDOW_START_Y);
            double range = sqrt(pow( calcPos.x - xCurrent ,2) + pow( calcPos.y - yCurrent ,2));

            double offset = abs( aLine*(x0+0.5) + bLine*(y0+0.5) + cLine )/abDist;

            if(x0 == x1 && y0 == y1){
              if(hitState){
                calculateCellHit(&complexOccopuncyGrid[y0][x0], range, offset);
              }
              break;
            }
            calculateCellMiss(&complexOccopuncyGrid[y0][x0], range, offset);
            int e2 = error*2;
            if(e2 >= dy){
              if(x0 == x1){
                break;
              }
              error = error + dy;
              x0 += sx;
            } 
            if(e2 <= dx){
              if(y0 == y1){
                break;
              }
              error = error + dx;
              y0 += sy;
            }
          }

      }

      pointClose();
      laserCount++;
      currentLaser = laserAdvance();
      if(currentLaser){
        keepLaser = currentLaser;
      }
    }

    currentVel = velAdvance();

#ifdef USE_GNU_PLOT
    printPos(GNU_COMB, time, calcPos.x, calcPos.y, calcPos.heading);
    if(!currentVel || (loopCount % GNU_UPDATE_FREQUENCY == 0)){
      printf("Save Occ\n");
      FILE * laserOcc = fopen(GNU_OCC_MATRIX, "w");
      for(int i = 0; i < WINDOW_ARRAY_Y; i++){
        fprintf(laserOcc, "%lf", complexOccopuncyGrid[i][0].hit);
        for(int ii = 1; ii < WINDOW_ARRAY_X; ii++){
          fprintf(laserOcc, " %lf", complexOccopuncyGrid[i][ii].hit);
        }
        fprintf(laserOcc, "\n");
      }
      fclose(laserOcc);

#if USE_GNU_PLOT == 1
      printf("Plotting\n");
      gnuplot_cmd(gnuPlot, plotCmd);
      gnuplot_cmd(occPlot, occCmd);
#endif //if USE_GNU_PLOT == 1
      
    }
#endif //ifdef USE_GNU_PLOT

#if GNU_SPEED_UP != 0
    printf("Sleep %lf\n",dt*1000000/GNU_SPEED_UP);
    usleep(dt*1000000/GNU_SPEED_UP);
#endif

    if(!currentVel){
      break;
    }
  }
  printf("Data Finished\n");


  //Techincally unnessary as memory is auto-freed on program end
  compClose();
  featClose();
  posClose();
  velClose();
  laserClose();
  printf("Enter to exit\n");
  char f = getchar();
#if USE_GNU_PLOT == 1
  gnuplot_close(gnuPlot);
#endif
  return 0;
}
