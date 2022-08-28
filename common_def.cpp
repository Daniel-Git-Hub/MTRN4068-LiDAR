#include "common_def.h"
#include <stdio.h>

typedef struct JUST_TIME_S {
    unsigned long second;
    unsigned int micro;
} JUST_TIME;

void clearComments(FILE * file){
    char fileReading[MAX_READ];

    int read;
    while(1){
        read = fgetc(file);
        ungetc(read, file);
        if(read != '%'){
            break;
        }
        fgets(fileReading, MAX_READ, file);
    }
}

int isRel(void * aRaw, void * bRaw){
    JUST_TIME * a = (JUST_TIME *)aRaw;
    JUST_TIME * b = (JUST_TIME *)bRaw;
    if(b->second < a->second){
        return 1;
    }
    if(b->second == a->second && b->micro <= a->micro){
        return 1;
    }


    return 0;
}

double computeSD(double * data, double avg, int sampleSize){
    double sum = 0;
    for(int i = 0; i < sampleSize; i++){
        sum += pow(data[i] - avg, 2);
    }
    return sum > 0 ? (sum/sampleSize) : -(sum/sampleSize);
}

int computeRealAvg(double * data, double * avg, int sampleSize){
    double sD = computeSD(data, *avg, sampleSize);
    double sum = 0;
    double count = 0;
    for(int i = 0; i < sampleSize; i++){
        if(data[i] >= (*avg) - OUTLIER_TOL*sD && data[i] <= (*avg) + OUTLIER_TOL*sD){
            sum += data[i];
            count++;
        }
    }
    if(!count){
        return 1;
    }
    *avg = sum/count;
    return 0;
}

double normAngle(double aim, double current){
    while(current > (aim + M_PI)){
      current -= 2*M_PI;
    }
    while(current < (aim - M_PI)){
      current += 2*M_PI;
    }
    return current;
}

double avgAngle(double *data, int sampleSize){
    double Y = 0;
    double X = 0;
    for(int i = 0; i < sampleSize; i++){
        X += cos(data[i]);
        Y += sin(data[i]);
    }
    return atan2(Y/sampleSize,X/sampleSize);

}

// int calculateCellHit(OCC_CELL * cell, double range){
//     double prob = 0.5 + 0.48*(LASER_MAX_RANGE - range)/LASER_MAX_RANGE;
//     // double prob = 0.9;

//     // double prob = 0.75 + 0.25*(LASER_MAX_RANGE - range)/LASER_MAX_RANGE;
//     // double prob = 1;
//     // double miss = 1 - cell->hit;
//     // cell->hit = (cell->hit * prob)/((cell->hit * prob) + (miss * (1 - prob)));
//     cell->hit = (cell->hit * prob)/((cell->hit * prob) + (cell->miss * (1 - prob)));
//     cell->miss = 1 - cell->hit;
//     return 0;
// }


// int calculateCellMiss(OCC_CELL * cell, double range){
//     double prob = 0.5 + 0.28*(LASER_MAX_RANGE - range)/LASER_MAX_RANGE;
//     // double prob = 0.9;
//     // double prob = 1;
//     // double miss = 1 - cell->hit;
//     // cell->hit = 1 - (miss * prob)/((miss * prob) + (cell->hit * (1 - prob)));
//     cell->miss = (cell->miss * prob)/((cell->miss * prob) + (cell->hit * (1 - prob)));
//     cell->hit = 1 - cell->miss;
//     return 0;
// }


int calculateCellHit(OCC_CELL * cell, double range, double offset){
    double prob = 0.98*((LASER_MAX_RANGE - range)/LASER_MAX_RANGE + (M_SQRT2 - offset)*M_SQRT1_2 )/2 ;
    if(prob < 0.5){
        prob = 0.5;
    }
    double miss = 1 - cell->hit ;
    cell->hit = (cell->hit * prob)/((cell->hit * prob) + (miss * (1 - prob)));
    return 0;
}


int calculateCellMiss(OCC_CELL * cell, double range, double offset){
    double prob = 0.98*((LASER_MAX_RANGE - range)/LASER_MAX_RANGE  + (M_SQRT2 - offset)*M_SQRT1_2 ) /2;

    if(prob < 0.5){
        prob = 0.5;
    }
    double miss = 1 - cell->hit;
    cell->hit = 1 - (miss * prob)/((miss * prob) + (cell->hit * (1 - prob)));
    return 0;
}


int printPos(char * fileName, double t, double h){
    FILE * file = fopen(fileName, "a");
    fprintf(file, "%.18e %.18e\n", t, h);
    fclose(file);
    return 0;
}
int printPos(char * fileName, double t, double x, double y){
    FILE * file = fopen(fileName, "a");
    fprintf(file, "%.18e %.18e %.18e\n", t, x, y);
    fclose(file);
    return 0;
}
int printPos(char * fileName, double t, double x, double y, double h){
    FILE * file = fopen(fileName, "a");
    fprintf(file, "%.18e %.18e %.18e %.18e\n", t, x, y, h);
    fclose(file);
    return 0;
}

int printFileInit(){
  FILE * file;
  
  file = fopen(GNU_RAW, "w");
  fprintf(file, "t x y h\n");
  fclose(file);
  
  file = fopen(GNU_COMPASS, "w");
  fprintf(file, "t h\n");
  fclose(file);
  
  file = fopen(GNU_GPS, "w");
  fprintf(file, "t x y\n");
  fclose(file);
  
  file = fopen(GNU_FEAT, "w");
  fprintf(file, "t x y h\n");
  fclose(file);
  
  file = fopen(GNU_COMB, "w");
  fprintf(file, "t x y h\n");
  fclose(file);


  file = fopen(GNU_OCC_MATRIX, "w");
  fprintf(file, "x y\n");
  fclose(file);


  file = fopen(GNU_FEAT_GRID, "w");
  fprintf(file, "x y\n");
  fclose(file);

  return 0;
}
