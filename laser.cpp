#include "laser.h"

#define LASER_DATA "laserObs.txt"

#define LASER_LINE_LENGTH 4096

FILE * laserFile;

LASER * laserData;
int laserMaxLength = 0;
int laserLength = 0;
int laserCurrent = 0;

int laserInit(){
    char fileName[FILE_PATH_LIMIT] = DATA_PATH;
    strcat(fileName, LASER_DATA);
    laserFile = fopen(fileName, "r");

    clearComments(laserFile);
    
    char laserLine[4096];

    while(1){
        if(laserMaxLength <= laserLength){
            if(laserMaxLength == 0){
                laserMaxLength = 4; //effectively starts the length at 8
            }
            laserMaxLength *= 2;
            laserData = (LASER *)realloc(laserData, sizeof(LASER)*laserMaxLength);
        }

        if(feof(laserFile)){
            break;
        }
        int result = fscanf(laserFile, "%lu %u", &laserData[laserLength].second, &laserData[laserLength].micro);
        if(result != 2){
            break;
        }
        
        for(int i = 0; i < LASER_POINTS; i++){
            result = fscanf(laserFile, "%lf %hhd", &laserData[laserLength].range[i], &laserData[laserLength].intensity[i]);
            
            if(result != 2){
                printf("Error");
                return 1;
            }    
        }

        laserLength++;
    }

    fclose(laserFile);

    return 0;
}

int laserClose(){
    free(laserData);
    return 0;
}

int laserPrintIndex(int i){
    printf("%lu.%u  -  ", laserData[i].second,laserData[i].micro);
    for(int ii = 0; ii < LASER_POINTS; ii++){
        printf("%lf %d, ", laserData[i].range[ii], laserData[i].intensity[ii]);
    }
    printf("\n");
    return 0;
}

LASER * laserGetCurrent(){
    if(laserCurrent < laserLength){
        return &laserData[laserCurrent];
    }
    return 0;
}

LASER * laserAdvance(){
    laserCurrent++;
    return laserGetCurrent();
}

