
#include "point.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "gnuplot_i.h"

#define IX (pointCloud[pointI].worldX)
#define IY (pointCloud[pointI].worldY)

#define FX (feat->x)
#define FY (feat->y)


int pointLength = 0;
int pointMaxLength = 1;
POINT_CLOUD * pointCloud = 0;


int pointClose(){
    if(pointCloud){
        for(int i = 0; i < pointLength; i++){
            free(pointCloud[i].features);
        }
        free(pointCloud);
    }
    pointMaxLength = 1;
    pointLength = 0;
    pointCloud = 0;
    return 0;
}

double pointDistanceCalc(POINT_CLOUD * cloud, FEATURE * feature){
    if(!cloud->featureCount){
        return -1;
    }
    double x = cloud->worldX - feature->x;
    double y = cloud->worldY - feature->y;
    return (x * x) + (y * y);
}

int pointUpdate(int idx){
    pointCloud[idx].worldX = 0;
    pointCloud[idx].worldY = 0;
    if(pointCloud[idx].featureCount){
        for(int i = 0; i < pointCloud[idx].featureCount; i++){
            pointCloud[idx].worldX += pointCloud[idx].features[i].x;
            pointCloud[idx].worldY += pointCloud[idx].features[i].y; 
        }
        pointCloud[idx].worldX = pointCloud[idx].worldX / pointCloud[idx].featureCount;
        pointCloud[idx].worldY = pointCloud[idx].worldY / pointCloud[idx].featureCount;   
    }
    return 0;
}



int pointAddFeature(int idx, FEATURE * feature){

    if(pointCloud[idx].featureCount >= (pointCloud[idx].maxFeatureCount - 1)){
        if(!pointCloud[idx].maxFeatureCount){
            pointCloud[idx].maxFeatureCount += 2;
        }

        pointCloud[idx].maxFeatureCount *= 2;
        pointCloud[idx].features = (FEATURE *)realloc(pointCloud[idx].features, sizeof(FEATURE)*pointCloud[idx].maxFeatureCount);
    }
    pointCloud[idx].features[pointCloud[idx].featureCount].x = feature->x;
    pointCloud[idx].features[pointCloud[idx].featureCount].y = feature->y;
    
    pointCloud[idx].featureCount++;
    pointUpdate(idx);

    return 0;
}

int pointAdd(FEATURE * feature){
    int closestCloud = -1;
    double minDistance = FEATURE_TOL;

    for(int i = 0; i < pointLength; i++){
        double dist = pointDistanceCalc(&pointCloud[i], feature);
        if(dist != -1 && dist < minDistance){
            closestCloud = i;
            minDistance = dist;
        }
    }

    if(closestCloud == -1){
        if(pointLength >= (pointMaxLength - 1)){
            pointMaxLength *= 2;
            pointCloud = (POINT_CLOUD *)realloc(pointCloud, sizeof(POINT_CLOUD)*pointMaxLength);

        }

        pointCloud[pointLength].featureCount = 0;
        pointCloud[pointLength].maxFeatureCount = 0;
        pointCloud[pointLength].features = 0;
        closestCloud = pointLength;
        pointLength++;
    }

    pointAddFeature(closestCloud, feature);

    return 0;
}

int pointPrintCloud(){
    FILE * laserFile = fopen("gnuPlot/laserCloud.csv", "w");
    fprintf(laserFile, "idx, x, y\n");
    for(int i = 0; i < pointLength; i++){
        for(int j = 0; j < pointCloud[i].featureCount; j++){
            fprintf(laserFile, "%d, %lf, %lf\n", i, pointCloud[i].features[j].x, pointCloud[i].features[j].y);
        }
    }
    fclose(laserFile);

    laserFile = fopen("gnuPlot/laserCluster.csv", "w");
    fprintf(laserFile, "idx, x, y\n");
    for(int pointJ = 0; pointJ < pointLength; pointJ++){
        fprintf(laserFile, "%d, %lf, %lf\n", pointJ, pointCloud[pointJ].worldX, pointCloud[pointJ].worldY);
    }
    fclose(laserFile);

    return 0;
}

BRUTE_RETURN pointBruteIter(CALC_POS * pos, CALC_POS  *finalTrans, int graph){
    BRUTE_RETURN result;

    double error = -1;
    int featureIdx = -1;
    int clusterIdx = -1;

    double tempX, tempY, temp2;

    int featureLength = featCount();
    
    if(pointLength < 3 || featureLength < pointLength){
        result.status = -1;
        return result;
    }
    
    double finalAngle = 0;

    for(int pointI = 0; pointI < pointLength; pointI++){
        for(int featI = 0; featI < featureLength; featI++){
            FEATURE * feat = featGet(featI);

            CALC_POS trans;
            
            //angle between the measured cluster point and the real point
            // trans.heading = atan2(FY, FX) - atan2(IY, IX); 
            trans.heading = 0;
            if(abs(trans.heading) > M_PI_4){ //assume the amount of change is not more then 45 degrees
                // continue;
            }

            // tempX = IX*cos(trans.heading) - IY*sin(trans.heading);
            tempX = IX;
            // tempY = IX*sin(trans.heading) + IY*cos(trans.heading);
            tempY = IY;

            trans.x = FX - tempX;
            trans.y = FY - tempY;
            double e = (trans.x * trans.x) + (trans.y * trans.y);
            e *= 0;
            double avgAngle = 0;
            
            for(int pointJ = 0; pointJ < pointLength; pointJ++){
                if(pointJ != pointI){
                    
                    tempX = pointCloud[pointJ].worldX + trans.x;
                    tempY = pointCloud[pointJ].worldY + trans.y;
                    
                    double tempAngle = -10;
                    double tempDistance = -1;
                    for(int i = 0; i < featureLength; i++){
                        FEATURE * tempClose = featGet(i);
                        
                        double angle2 = atan2(tempClose->y - FY, tempClose->x - FX) - atan2(tempY - FY, tempX - FX);
                        while(angle2 > M_PI_2){
                            angle2 -= M_PI;
                        }
                        while(angle2 < -M_PI_2){
                            angle2 += M_PI;
                        }
                        
                        double temp2X = tempX - FX;
                        double temp2Y = tempY - FY;
                        double temp3X = temp2X*cos(angle2) - temp2Y*sin(angle2);
                        double temp3Y = temp2X*sin(angle2) + temp2Y*cos(angle2);
                        temp2X = temp3X + FX;
                        temp2Y = temp3Y + FY;

                        double dist = (tempClose->x - temp2X)*(tempClose->x - temp2X) + (tempClose->y - temp2Y)*(tempClose->y - temp2Y);

                        if(tempDistance == -1 || dist < tempDistance){
                            tempDistance = dist;
                            tempAngle = angle2;
                        }    
                    }
                    
                    //This is the same as the above for loop but only looks at the closest feature
                    // FEATURE * tempClose = featGet(featGetClosest(tempX,tempY));
                    
                    // double tempAngle = atan2(tempClose->y - FY, tempClose->x - FX) - atan2(tempY - FY, tempX - FX);

                    // while(tempAngle > M_PI_2){
                    //     tempAngle -= M_PI;
                    // }
                    // while(tempAngle < -M_PI_2){
                    //     tempAngle += M_PI;
                    // }


                    avgAngle += tempAngle;

                    for(int pointK = 0; pointK < pointLength; pointK++){
                        
                        temp2 = trans.x + pointCloud[pointK].worldX;
                        tempY = trans.y + pointCloud[pointK].worldY; 


                        temp2 -= FX;
                        tempY -= FY;

                        tempX = temp2*cos(tempAngle) - tempY*sin(tempAngle);
                        tempY = temp2*sin(tempAngle) + tempY*cos(tempAngle); 
                        
                        tempX += FX;
                        tempY += FY;


                        //strictly speaking this should be the sum of the sqrt. But the benefit that brings is not worth the cost of sqrt
                        e += featGetClosestDistance(tempX, tempY);
                    }

                    if(error == -1 || e < error){
                        error = e;
                        // *finalTrans = trans;
                        finalTrans->x = trans.x;
                        finalTrans->y = trans.y;
                        finalTrans->heading = trans.heading;
                        featureIdx = featI;
                        clusterIdx = pointI;
                        finalAngle = tempAngle;
                    }


                }
            }

        }
    }

#ifdef USE_GNU_PLOT
    //This plots the an instance of the logic behind a LiDAR scan, as shown in Figure2
    if(graph){
        FILE * laserFile;

        FEATURE * feat = featGet(featureIdx);

        laserFile = fopen("gnuPlot/laserTran.csv", "w");
        fprintf(laserFile, "idx, tranX, tranY, rotateX, rotateY\n");
        for(int pointJ = 0; pointJ < pointLength; pointJ++){
            temp2 = finalTrans->x + pointCloud[pointJ].worldX;
            tempY = finalTrans->y + pointCloud[pointJ].worldY; 

            fprintf(laserFile, "%d, %lf, %lf, ", pointJ, temp2, tempY);


            temp2 -= FX;
            tempY -= FY;

            tempX = temp2*cos(finalAngle) - tempY*sin(finalAngle);
            tempY = temp2*sin(finalAngle) + tempY*cos(finalAngle); 
            tempX += FX;
            tempY += FY;

            fprintf(laserFile, "%lf, %lf\n", tempX, tempY);
        }
        fclose(laserFile);
    }
#endif
    result.status = 0;
    result.angle = finalAngle;
    result.featureIdx = featureIdx;

    return result;
}



//Unused
int pointBrute(CALC_POS * pos, CALC_POS  *finalTrans, int graph){
    double error = -1;
    int featureIdx = -1;
    int clusterIdx = -1;


    int featureLength = featCount();
    
    if(pointLength < 3 || featureLength < pointLength){
        return -1;
    }
    

    for(int pointI = 0; pointI < pointLength; pointI++){
        for(int featI = 0; featI < featureLength; featI++){
            FEATURE * feat = featGet(featI);

            CALC_POS trans;
            
            //angle between the measured cluster point and the real point
            trans.heading = atan2(FY, FX) - atan2(IY, IX); 

            if(abs(trans.heading) > M_PI_4){ //assume the amount of change is not more then 45 degrees
                continue;
            }

            double tempX = IX*cos(trans.heading) - IY*sin(trans.heading);
            double tempY = IX*sin(trans.heading) + IY*cos(trans.heading);

            trans.x = FX - tempX;
            trans.y = FY - tempY;
            double e = 0;

            for(int pointJ = 0; pointJ < pointLength; pointJ++){
                // double tempX = pointCloud[pointJ].worldX + trans.x;
                // double tempY = pointCloud[pointJ].worldY + trans.y;
                
                tempX = pointCloud[pointJ].worldX*cos(trans.heading) - pointCloud[pointJ].worldY*sin(trans.heading);
                tempY = pointCloud[pointJ].worldX*sin(trans.heading) + pointCloud[pointJ].worldY*cos(trans.heading); 

                tempX = tempX + trans.x;
                tempY = tempY + trans.y;


                //strictly speaking this should be the sum of the sqrt. But the benefit that brings is not worth the cost of sqrt
                e += featGetClosestDistance(tempX, tempY);
            }

            if(error == -1 || e < error){
                error = e;
                // *finalTrans = trans;
                finalTrans->x = trans.x;
                finalTrans->y = trans.y;
                finalTrans->heading = trans.heading;
                featureIdx = featI;
                clusterIdx = pointI;
            }

        }
    }

    if(graph & 2){
        FILE * laserFile;
        laserFile = fopen("result/laserTran.csv", "w");
        fprintf(laserFile, "idx, x, y\n");
        for(int pointJ = 0; pointJ < pointLength; pointJ++){
            double tempX = finalTrans->x + pointCloud[pointJ].worldX*cos(finalTrans->heading) - pointCloud[pointJ].worldY*sin(finalTrans->heading);
            double tempY = finalTrans->y + pointCloud[pointJ].worldX*sin(finalTrans->heading) + pointCloud[pointJ].worldY*cos(finalTrans->heading); 

            fprintf(laserFile, "%d, %lf, %lf\n", pointJ, tempX, tempY);
        }
        fclose(laserFile);
    }



    return 0;
}

