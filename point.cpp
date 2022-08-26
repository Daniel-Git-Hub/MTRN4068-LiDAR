
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

gnuplot_ctrl * h1;

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
        }
        pointCloud = (POINT_CLOUD *)realloc(pointCloud, sizeof(POINT_CLOUD)*pointMaxLength);

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
    FILE * laserFile = fopen("result/laserCloud.csv", "w");
    fprintf(laserFile, "idx, x, y\n");
    for(int i = 0; i < pointLength; i++){
        for(int j = 0; j < pointCloud[i].featureCount; j++){
            fprintf(laserFile, "%d, %lf, %lf\n", i, pointCloud[i].features[j].x, pointCloud[i].features[j].y);
        }
    }
    fclose(laserFile);

    laserFile = fopen("result/laserCluster.csv", "w");
    fprintf(laserFile, "idx, x, y\n");
    for(int pointJ = 0; pointJ < pointLength; pointJ++){
        fprintf(laserFile, "%d, %lf, %lf\n", pointJ, pointCloud[pointJ].worldX, pointCloud[pointJ].worldY);
    }
    fclose(laserFile);

    return 0;
}


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

    if(graph & 1){
        h1 = gnuplot_init();
        char robot[] = "Robot";
        double tempXGraph[] = {pos->x};
        double tempYGraph[] = {pos->y};
        gnuplot_plot_xy(h1, tempXGraph, tempYGraph, 1, "Robot");

        char untran[] = "Untransformed";

        
        gnuplot_plot_xy_point(h1, pointCloud, pointLength, untran);

        for(int pointJ = 0; pointJ < pointLength; pointJ++){
            pointCloud[pointJ].worldX = finalTrans->x + pointCloud[pointJ].worldX*cos(finalTrans->heading) - pointCloud[pointJ].worldY*sin(finalTrans->heading);
            pointCloud[pointJ].worldY = finalTrans->y + pointCloud[pointJ].worldX*sin(finalTrans->heading) + pointCloud[pointJ].worldY*cos(finalTrans->heading); 
        }

        char tran[] = "Transformed";
        gnuplot_plot_xy_point(h1, pointCloud, pointLength, tran);

        char feat[] = "Features";
        gnuplot_plot_xy_feat(h1, featGet(0), featCount(), tran);
    }



    return 0;
}

int pointCloseGnu(){
    if(h1){
        gnuplot_close(h1);
    }
    return 0;
}
