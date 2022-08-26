#include "vel.h"
#include "common_def.h"

#define VEL_DATA "velocityObs.txt"


FILE * velFile;

VEL * velData;
int velMaxLength = 0;
int velLength = 0;
int velCurrent = 0;


int velInit(){
    char fileName[FILE_PATH_LIMIT] = DATA_PATH;
    strcat(fileName, VEL_DATA);
    velFile = fopen(fileName, "r");

    clearComments(velFile);
    
    while(1){
        if(velMaxLength <= velLength){
            if(velMaxLength == 0){
                velMaxLength = 4; //effectively starts the length at 8
            }
            velMaxLength *= 2;
            velData = (VEL *)realloc(velData, sizeof(VEL)*velMaxLength);
        }
        int result = fscanf(velFile, "%lu %u %lf %lf", &velData[velLength].second, &velData[velLength].micro, &velData[velLength].vel, &velData[velLength].turnRate);
        
        if(result != 4 || feof(velFile)){
            break;
        }
        velLength++;
    }

    fclose(velFile);

    return 0;
}

int velClose(){
    free(velData);
    return 0;
}

VEL * velGetCurrent(){
    if(velCurrent < velLength){
        return &velData[velCurrent];
    }
    return 0;
}

double velGetDt(){
    int ind = velCurrent;
    if(velCurrent == 0){
        ind++;
    }
    double dt = (velData[ind].second - velData[ind-1].second);
    dt += (((signed long)velData[ind].micro) - velData[ind-1].micro)*(0.000001);
    return dt;
}

VEL * velAdvance(){
    velCurrent++;
    return velGetCurrent();
}

