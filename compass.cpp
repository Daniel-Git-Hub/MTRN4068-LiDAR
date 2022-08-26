#include "compass.h"
#include "common_def.h"
#include <math.h>


#define COMPASS_DATA "compassObs.txt"


FILE * compFile;

COMP * compData;
int compMaxLength = 0;
int compLength = 0;
int compCurrent = 0;


int compInit(){
    char fileName[FILE_PATH_LIMIT] = DATA_PATH;
    strcat(fileName, COMPASS_DATA);
    
    compFile = fopen(fileName, "r");

    clearComments(compFile);
    while(1){
        if(compMaxLength <= compLength){
            if(compMaxLength == 0){
                compMaxLength = 4; //effectively starts the length at 8
            }
            compMaxLength *= 2;
            compData = (COMP *)realloc(compData, sizeof(COMP)*compMaxLength);
        }
        int result = fscanf(compFile, "%lu %u %lf", &compData[compLength].second, &compData[compLength].micro, &compData[compLength].heading);
        if(result != 3 || feof(compFile)){
            break;
        }
        // if(compData[compLength].heading < 0){
        //     compData[compLength].heading += 2*M_PI;
        // }else if(compData[compLength].heading > 2*M_PI){
        //     compData[compLength].heading -= 2*M_PI;
        // }
        compLength++;
    }

    fclose(compFile);

    return 0;
}

int compClose(){
    free(compData);
    return 0;
}

COMP * compGetCurrent(){
    if(compCurrent < compLength){
        return &compData[compCurrent];
    }
    return 0;
}

COMP * compAdvance(){
    compCurrent++;
    return compGetCurrent();
}