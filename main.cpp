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


int print_pos(CALC_POS);
int fillOccGrid(LASER *, CALC_POS *);

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
"set pointsize 5\n"
// "set title 'Occupancy Grid'\n"
"plot '" GNU_COMB "' volatile using 'x':'y' title 'Combined' with lines, "
"'" GNU_OCC_GRID "' volatile using 'x':'y' title 'GPS' with points pt '⊙', "
"'" GNU_FEAT_GRID "' volatile using 'x':'y' title 'GPS' with points pt '⊙'\n"
;

  printf("Init start\n");



#ifdef USE_GNU_PLOT
  printFileInit();

  gnuplot_ctrl * gnuPlot, * occPlot;
  gnuPlot = gnuplot_init();
  gnuplot_cmd(gnuPlot, plotCmd);
  
  occPlot = gnuplot_init();
  gnuplot_cmd(occPlot, occCmd);
#endif

  compInit();

  featInit();

  posInit();

  velInit();

  laserInit();

  for(int i = 0; i < WINDOW_ARRAY_Y; i++){
    for(int ii = 0; ii < WINDOW_ARRAY_X; ii++){
      complexOccopuncyGrid[i][ii].hit = 0.5;
      complexOccopuncyGrid[i][ii].miss = 0.5;
    }

  }

  FILE * posResult = fopen(POS_RESULT_FILE, "w");
  fprintf(posResult, "time, gpsX, gpsY, compass, velX, velY, velHeading, combX, combY, combHeading, featX, featY, featHeading\n");
  
  memset(occopuncyGrid, 0, (WINDOW_ARRAY_Y)*(WINDOW_ARRAY_X));

  // printf("%d\n", featPrintAll());  
  // laserPrintIndex(0);
  // laserPrintIndex(20);

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


      if((laserCount) == 3){
        printf("Lasercount %lf, %lf, %lf\n", featPos.x, featPos.y, featPos.heading);
        FILE * laserFile = fopen("result/laserRobotPos.csv", "w");
        fprintf(laserFile, "idx, x, y, heading\n");
        fprintf(laserFile, "%d, %lf, %lf, %lf\n", 0, calcPos.x, calcPos.y, calcPos.heading);
        fclose(laserFile);

        pointPrintCloud();
      }

      if(!pointBrute(&calcPos, &featPos, (laserCount == 3) ? 2 : 0)){
        // double tempX = featPos.x + calcPos.x;
        // double tempY = featPos.y + calcPos.y;
        double tempX = featPos.x + calcPos.x*cos(featPos.heading) - calcPos.y*sin(featPos.heading);
        double tempY = featPos.y + calcPos.x*sin(featPos.heading) + calcPos.y*cos(featPos.heading); 
        featPos.heading = normAngle(calcPos.heading, featPos.heading + calcPos.heading);


        featPos.x = tempX;
        featPos.y = tempY;

#ifdef USE_GNU_PLOT
        printPos(GNU_FEAT, time, featPos.x, featPos.y, featPos.heading);
#endif

        if(abs(featPos.x - calcPos.x) <= FEATURE_OUTLIER_TOL && abs(featPos.y - calcPos.y) <= FEATURE_OUTLIER_TOL){
          calcPos.heading = FEAT_HEAD_GAIN*calcPos.heading + (1-FEAT_HEAD_GAIN)*(featPos.heading);
          calcPos.x = FEAT_GAIN*calcPos.x + (1-FEAT_GAIN)*tempX;
          calcPos.y = FEAT_GAIN*calcPos.y + (1-FEAT_GAIN)*tempY;
        }

        // calcPos.heading = FEAT_HEAD_GAIN*calcPos.heading + (1-FEAT_HEAD_GAIN)*(calcPos.heading + featPos.heading);

      }


      //compute occupancy grid
      for(int x = 0; x < LASER_POINTS; x++){
        double laserAngle = LASER_START_ANGLE + x*LASER_ANGLE_RANGE/(LASER_POINTS-1);

        int closestX;
        int closestY;
        if(currentLaser->range[x] < LASER_MAX_RANGE){
          double hitX = calcPos.x + cos(laserAngle+calcPos.heading)*currentLaser->range[x];
          double hitY = calcPos.y + sin(laserAngle+calcPos.heading)*currentLaser->range[x];
          

          closestX = (int)((hitX-WINDOW_START_X)*WINDOW_RESOLTION);
          closestY = (int)((hitY-WINDOW_START_Y)*WINDOW_RESOLTION);

          if(closestX < 0 || closestX >= ((int)WINDOW_SIZE_X*WINDOW_RESOLTION) || closestY < 0 || closestY >= ((int)WINDOW_SIZE_Y*WINDOW_RESOLTION)){
            continue;
          }else{
            calculateCellHit(&complexOccopuncyGrid[closestY][closestX], currentLaser->range[x]);
          }

          // int test = 1;

          //A better raytracing algorihtm should/could be used
          //This basically looks for all the grid cells that we can assume are empty (i.e between the robot and a hit target)
          //Also I should do this for itensity is 0
        }

          int lastX = -1;
          int lastY = -1;
          for(double smallRange = 0; smallRange < currentLaser->range[x]; smallRange += RANGE_INC){
            double emptyX = calcPos.x + cos(laserAngle+calcPos.heading)*smallRange;
            double emptyY = calcPos.y + sin(laserAngle+calcPos.heading)*smallRange;

            int closeEmptyX = (int)((emptyX-WINDOW_START_X)*WINDOW_RESOLTION);
            int closeEmptyY = (int)((emptyY-WINDOW_START_Y)*WINDOW_RESOLTION);
            if((currentLaser->intensity[x] && closeEmptyX == closestX && closeEmptyY == closestY)){
              break;
            }
            if((closeEmptyX == lastX && closeEmptyY == lastY)){
              // test = 1;
              continue;
            }
            if(closeEmptyX >= 0 && closeEmptyX < ((int)WINDOW_SIZE_X*WINDOW_RESOLTION) && closeEmptyY >= 0 && closeEmptyY < ((int)WINDOW_SIZE_Y*WINDOW_RESOLTION)){
              calculateCellMiss(&complexOccopuncyGrid[closeEmptyY][closeEmptyX], smallRange);
              lastX = closeEmptyX;
              lastY = closeEmptyY;
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




    fprintf(posResult, "%lf, ", time);
    fprintf(posResult, "%lf, %lf, ", keepPos->x, keepPos->y);
    fprintf(posResult, "%lf, ", normHeading);
    fprintf(posResult, "%lf, %lf, %lf, ", rawPos.x, rawPos.y, rawPos.heading);
    fprintf(posResult, "%lf, %lf, %lf, ", calcPos.x, calcPos.y, calcPos.heading);
    fprintf(posResult, "%lf, %lf, %lf\n", featPos.x, featPos.y, featPos.heading);
    

    currentVel = velAdvance();

#ifdef USE_GNU_PLOT
      printPos(GNU_COMB, time, calcPos.x, calcPos.y, calcPos.heading);

      if(!currentVel || (loopCount % GNU_UPDATE_FREQUENCY == 0)){
        FILE * file = fopen(GNU_OCC_GRID, "w");
        fprintf(file, "x y\n");
        for(int i = 0; i < WINDOW_ARRAY_Y; i++){
          for(int ii = 1; ii < WINDOW_ARRAY_X; ii++){
            if(complexOccopuncyGrid[i][ii].hit >= OCC_THRESHOLD){
              double tempX = (((double)ii)+0.5)/WINDOW_RESOLTION + WINDOW_START_X;
              double tempY = (((double)i)+0.5)/WINDOW_RESOLTION + WINDOW_START_Y;
              fprintf(file, "%.18e %.18e\n", tempX, tempY);
            }
          }
        }
        fclose(file);

        gnuplot_cmd(gnuPlot, plotCmd);
	gnuplot_cmd(occPlot, occCmd);
        
      }
#ifdef GNU_SPEED_UP      
      usleep(dt*1000000/GNU_SPEED_UP);
#endif

#endif


    if(!currentVel){
      break;
    }

  }
  printf("Data Finished\n");

  FILE * laserOcc = fopen("result/laserOcc.csv", "w");
  for(int i = 0; i < WINDOW_ARRAY_Y; i++){
    fprintf(laserOcc, "%d", occopuncyGrid[i][0]);
    for(int ii = 1; ii < WINDOW_ARRAY_X; ii++){
      fprintf(laserOcc, ", %lf", complexOccopuncyGrid[i][ii].hit);
    }
    fprintf(laserOcc, "\n");
  }
  fclose(laserOcc);

  //Techincally unnessary as memory is auto-freed on program end
  compClose();
  featClose();
  posClose();
  velClose();
  laserClose();
  printf("Enter to exit\n");
  char f = getchar();
#ifdef USE_GNU_PLOT
  gnuplot_close(gnuPlot);

#endif
  return 0;
}
