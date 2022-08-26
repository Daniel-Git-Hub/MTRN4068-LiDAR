#include "feature.h"
#include <math.h>

#define FEATURE_DATA "laserFeatures.txt"

int featureLength = 0;
int featureMaxLength = 0;
FEATURE * featureData;



int featInit(){
    char fileName[FILE_PATH_LIMIT] = DATA_PATH;
    strcat(fileName, FEATURE_DATA);
    FILE * featFile = fopen(fileName, "r");


    clearComments(featFile);
    
    while(1){
        if(featureMaxLength <= featureLength){
            if(featureMaxLength == 0){
                featureMaxLength = 4;
            }
            featureMaxLength *= 2;
            featureData = (FEATURE *)realloc(featureData, sizeof(FEATURE)*(featureMaxLength));

        }

        int result = fscanf(featFile, "%lf %lf", &featureData[featureLength].x, &featureData[featureLength].y);
        if(result != 2){
            break;
        }
        printPos(GNU_FEAT_GRID, featureData[featureLength].x, featureData[featureLength].y);
        featureLength++;
    }
    
    fclose(featFile);
    return 0;
}

int featClose(){
    free(featureData);
    return 0;
}

int featPrintAll(){
    for(int i = 0; i < featureLength; i++){
        printf("%lf, %lf\n", featureData[i].x, featureData[i].y);
    }
    return featureLength;
}

FEATURE * featGet(int idx){
    return &featureData[idx];
}

int featGetClosest(double x, double y){
    double distance = FEATURE_TOL; //the square of the min distance allowed for it to be counted as that feature
    int result = -1;

    for(int i = 0; i < featureLength; i++){
        double xDif = featureData[i].x - x;
        double yDif = featureData[i].y - y;
        double temp = (xDif * xDif) + (yDif * yDif);

        if(temp < distance){
            distance = temp;
            result = i;
        }
    }

    return result;
}

double featGetClosestDistance(double x, double y){
    double distance = -1; //the square of the min distance allowed for it to be counted as that feature

    for(int i = 0; i < featureLength; i++){
        double xDif = featureData[i].x - x;
        double yDif = featureData[i].y - y;
        double temp = (xDif * xDif) + (yDif * yDif);

        if(distance == -1 || temp < distance){
            distance = temp;
        }
    }

    return distance;
}


int featGetClosest(CALC_POS * pos){
    return featGetClosest(pos->x, pos->y);
}

int featCount(){
    return featureLength;
}

double featGetDistance(int i, double x, double y){
    x = featureData[i].x - x;
    y = featureData[i].y - y;
    return (x*x) + (y*y);
}