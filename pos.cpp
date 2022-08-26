#include "pos.h"
#include "common_def.h"

#define POS_DATA "positionObs_updated.txt"

FILE * posFile;

POS * posData;

int posMaxLength = 0;
int posLength = 0;
int posCurrent = 0;


int posInit(){
    char fileName[FILE_PATH_LIMIT] = DATA_PATH;
    strcat(fileName, POS_DATA);
    posFile = fopen(fileName, "r");

    clearComments(posFile);
    
    while(1){
        if(posMaxLength <= posLength){
            if(posMaxLength == 0){
                posMaxLength = 4; //effectively starts the length at 8
            }
            posMaxLength *= 2;
            posData = (POS *)realloc(posData, sizeof(POS)*posMaxLength);
        }
        int result = fscanf(posFile, "%lu %u %lf %lf", &posData[posLength].second, &posData[posLength].micro, &posData[posLength].x, &posData[posLength].y);

        // printf("%lu %u %lf %lf\n", posData[posLength].second, posData[posLength].micro, posData[posLength].x, posData[posLength].y);

        if(result != 4 || feof(posFile)){
            break;
        }
        posLength++;
    }

    fclose(posFile);

    return 0;
}

int posClose(){
    free(posData);
    return 0;
}

POS * posGetCurrent(){
    if(posCurrent < posLength){
        return &posData[posCurrent];
    }
    return 0;
}

POS * posAdvance(){
    posCurrent++;
    return posGetCurrent();
}

