#include "../rplidar_rpi/a1lidarrpi.h"
#include "environment.h"
#include <stdio.h>
#include <stdlib.h>
#include <string> 
#include <iomanip>
#include <sstream> //for writing string into file, for checking, std::ostringstream
#include <iostream>

//TO DO: benchmark

class DataInterface : public A1Lidar::DataInterface{
public: 
    int numberOfScans=0; //for testing purposes
    Box2DEnv * box2d;
    

    void setBox2D(Box2DEnv * _box2d){
        box2d = _box2d;
    }


	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
        lnumberOfScans++;
        int obstacleIndexCount = 0;
        box2d->createMap();
        Frame * frame = box2d->frames->at(box2d->frames->size()-1);
        frame->coordinates.resize(240*240); //range is 12m
        cv::Point point;
        // std::stringstream tmp;
        // tmp << "map" << std::setw(4) << std::setfill('0') << numberOfScans << ".dat";
        // const char * filename = tmp.str().c_str();
        // std::cout<<filename<<std::endl;
        //FILE * file =fopen(filename, "w+"); //uncomment
        float x, y;
        int iteration =0;
        std::cout<<"Scan n. "<<numberOfScans<<"#############################################################################################################"<<std::endl;
		for (A1LidarData &data:data){
			if (data.valid){ 
                //std::ostringstream stream;
                Obstacle* obstacle = new Obstacle(*(box2d->world), x, y); //doesn't check obstacles for duplicates
			    frame->obstacles->push_back(obstacle);
                point = cv::Point(approximate(data.x), approximate(data.y));
                frame->coordinates[iteration] = point;
                // //stream <<obstacle->body->GetPosition().x<<"\t"<<obstacle->body->GetPosition().y<<"\n";
                // //std::cout<<obstacle->body->GetPosition().x<<"\t"<<obstacle->body->GetPosition().y<<"\n";
                // //const char * line = stream.str().c_str();
                //  //fputs(line, file);
			}
            iteration++;
		}
        ////fclose(file);
        box2d->simulate(); //simulates collision for 5 seconds
        std::cout << "velocity estimated to be "<<box2d->realVelocity.x<<", "<<box2d->realVelocity.y<<"\n";

	}


	float approximate(float a){ //more efficient with pointers?? (float a, float * ptr){assign value of a to *ptr}
		a = round(a*10)/10;
		if (a<0){
			a-=0.05;
		}
		else if (a>=0){
			a+= 0.05;
		}
        return a;
	}
    
    // void transformForMatrix(float x, float y, int i, int j){ //transform cartesian coordinates into matrix coordinates
    //     //matrix is 240 bins of .1m, lidar data is in meters
    //     x = int(x*10); // transform x and y in decimeters and rounds down
    //     y = int(y*10);
    //     i= 240/2-y; // x index: columns/2-y
    //     j= 240/2+x; //y index: columns/2+x

    // }
};

