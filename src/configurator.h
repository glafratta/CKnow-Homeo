#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include "Box2D/Box2D.h"
#include "robot.h"
#include <dirent.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include <thread>
#include <filesystem>
#include <cmath>
#include <unistd.h>
#include <ncurses.h>
#include <fstream>
#include "state.h"
#include "plan.h"
#include <time.h>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, State> Graph;
typedef std::pair<State &, State&> Edge; 
// typedef boost::graph_traits<Graph>::vertex_iterator vIt; 
typedef boost::graph_traits<Graph>::vertex_descriptor vD;


struct Point{
	float x=0;
	float y=0;
	float r=0;
	float phi=0;

	Point(){}

	Point(float _x, float _y): x(_x), y(_y){
		r= sqrt(x*x+y*y);
		phi = atan(y/x);
	}

	Point(b2Vec2 v): x(v.x), y(v.y){
		r= sqrt(x*x+y*y);
		phi = atan(y/x);
	}

	Point(float _x, float _y, float _r, float _phi): x(_x), y(_y), r(_r), phi(_phi){}

	void operator=(const Point &p){
		x = p.x;
		y= p.y;
		r= p.r;
		phi = p.phi;
	}

	bool operator==(Point &p){
		return (x == p.x && y == p.y);
	}

	bool operator!=(Point &p){
		if (*this == p){
			return false;
		}
		else if (!(*this ==p)){
			return true;
		}
	}

	bool isInSquare(b2Vec2 point, float radius = 0.05){
		if (this->x < point.x+radius && this->x >point.y-radius && this->y < point.y+radius && this->y >point.y-radius){
			return true;
		}
		else{
			return false;
		}
	}

	b2Vec2 getb2Vec2(){
		return b2Vec2(x,y);
	}

};


class Configurator{
protected:
	//std::vector <float> timeStamps;
	double samplingRate = 1.0/ 5.0; //default
	int iteration=0; //represents that hasn't started yet, robot isn't moving and there are no map data
	bool crashed=0;
	b2Vec2 desiredVelocity;
	b2Vec2 absPosition = {0.0f, 0.0f};
	FILE * dumpPath;
	FILE * dumpDeltaV;
	char fileNameBuffer[50];
	int maxObstacleWM =3;
	State state;
public:
	float affineTransError =0;
	bool filterOn=1;
	char *folder;
	char readMap[50];
	char msg[25];
	State desiredState;
	std::chrono::high_resolution_clock::time_point previousTimeScan;
	float timeElapsed =0;
	float totalTime=0;
	std::vector <Point> current;

	struct getVelocityResult{
		bool valid =0;
		b2Vec2 vector = {0.0f, 0.0f};
		float angle;
		getVelocityResult(){}
		getVelocityResult(b2Vec2 disp, float maxSpeed = 0.125):vector(disp){
			valid=1;
			if (disp.y ==0 && disp.x==0){
				angle =0;
			}
			else{
				angle= atan(disp.y/disp.x);
			}
		}

	};

//calculuate displacement and angle using partial affine transformation

// 1:  new scan available: box2d world is rebuilt with objects, current Action is checked#

Configurator(){
	previousTimeScan = std::chrono::high_resolution_clock::now();
	totalTime = 0.0f;
	state = desiredState;
	//plan.states.push_back(desiredState);
	//leftWheelSpeed = desiredState.getAction().getLWheelSpeed();
	//rightWheelSpeed = desiredState.getAction().getRWheelSpeed();
	dumpDeltaV = fopen("/tmp/deltaV.txt", "w");

}

Configurator(State &_state): desiredState(_state), state(_state){
	previousTimeScan = std::chrono::high_resolution_clock::now();
	//plan.states.push_back(desiredState);
	//state = _state;
	//leftWheelSpeed = desiredState.getAction().getLWheelSpeed();
	//rightWheelSpeed = desiredState.getAction().getRWheelSpeed();
	totalTime =0.0f;

}

void __init__(){
}

void __init__(State & _state){
}

class MedianFilter{
    int kSize;
    std::vector <float> bufferX;
    std::vector <float> bufferY;
public:
    MedianFilter(int _k=3): kSize(_k){
        bufferX = std::vector<float>(kSize);
        bufferY = std::vector<float>(kSize);
    }

    void filterFloat(float &c, std::vector<float> & buffer){ //value gets modified in the input vector
        buffer.push_back(c);
        buffer.erase(buffer.begin());
        std::vector <float> tmp = buffer;
        std::sort(tmp.begin(), tmp.end());
        int index = int(buffer.size()/2)+1;
        c = tmp[index];
        //printf("median value at index: %i, value = %f\n", index, tmp[index]);
    }

    void applyToPoint(cv::Point2f &p){
        filterFloat(p.x, bufferX);
        filterFloat(p.y, bufferY);
    }

};

void setNameBuffer(char * str){ //set name of file from which to read trajectories. by default trajectories are dumped by 'state' into a robot000n.txt file.
								//changing this does not change where trajectories are dumped, but if you want the robot to follow a different trajectory than the one created by the state
	sprintf(fileNameBuffer, "%s", str);
	printf("reading trajectories from: %s\n", fileNameBuffer);
}

void setReadMap(char * str){
	sprintf(readMap,"%s", str);
	printf("map name: %s\n", readMap);

}

char * getReadMap(){
	return readMap;
}

char * getFolder(){
	return folder;
}
void setFolder(char * _folder){ //the folder from where LIDAR data is read
	std::filesystem::path folderPath(_folder);
		if (exists(folderPath)){
			if (is_directory(folderPath)){
				folder = _folder;
			}
			else{
				printf("not a directory");
			}
		}
		else{
			printf("%s doesn't exist", _folder);
		}
	printf("maps stored in %s\n", folder);
}

// void NewScan(std::vector <cv::Point2f> &); 

void NewScan(std::vector <Point> &); 

int getIteration(){
	return iteration;
}
// Configurator::getVelocityResult GetRealVelocity(std::vector <cv::Point2f> &, std::vector <cv::Point2f> &);

Configurator::getVelocityResult GetRealVelocity(std::vector <Point> &, std::vector <Point> &);

void controller();

void addIteration(){
	iteration++;
}


void updateAbsPos(b2Vec2 vel){
	absPosition.x += vel.x*timeElapsed;
	absPosition.y += vel.y*timeElapsed;
}

b2Vec2 getAbsPos(){
	return absPosition;
}

// Plan * plan(){
// 	return &currentPlan;
// }


State * getState(int advance=0){ //returns state being executed
	return &state;
}


void applyController(bool, State &);

int getMaxObstacleWM(){
	return maxObstacleWM;
}

b2Vec2 estimateDisplacementFromWheels();

int getMaxStepDuration(){
	return state.hz * state.getSimDuration();
}

State returnBest(State & s1, State & s2){ //returns pointer, remember to dereference
	switch(s1.getType() == s2.getType()){
	//if one of the state is the desired state, return the desired state
		case 0: 
			if (s1.getType() == desiredState.getType()){
				return s1;
			}
			else if (s2.getType() == desiredState.getType()){
				return s2;
			}
		break;
		case 1:
			switch (s1.getType()== desiredState.getType()){
				//if both states are avoiding, choose the one with the least duration
				case 0: 
					if (s1.getStepDuration()< s2.getStepDuration()){
						return s1;
					}
					else if (s2.getStepDuration()< s1.getStepDuration()){
						return s2;
					}
					else {
						return s1;
					}
				//if both states are desired, choose the one with the most
				case 1:
					if (s1.getStepDuration()> s2.getStepDuration()){
						return s1;
					}
					else if (s2.getStepDuration()> s1.getStepDuration()){
						return s2;
					}
					else {
						return s1;
					}
				default:
					break;
			}

			break;
			default: break;
	}

}

void eliminateDisturbance(b2World &, State &, b2Vec2&, float&, State::Direction); //performs reactive avoidance

void eliminateDisturbance(b2World &, vD &, Graph &, b2Vec2 &, float &, State::Direction);

void decisionTreeAvoidance();

};




 #endif