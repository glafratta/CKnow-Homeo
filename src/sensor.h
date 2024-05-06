#ifndef SENSOR_H
#define SENSOR_H
#include "task.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>

class Configurator;

class Pointf: public cv::Point2f{
	public: 

	Pointf(){}

	Pointf(float _x, float _y){
		x=_x;
		y=_y;
	}
    
    Pointf operator+(const Pointf p){
			Pointf result;
			result.x = x + p.x;
			result.y = y+ p.y;
			return result;
	}

};

template<>
struct cv::traits::Depth<Pointf> {enum {value = Depth<cv::Point2f>::value};};

template<>
struct cv::traits::Type<Pointf> {enum {value = CV_MAKETYPE(cv::traits::Depth<Pointf>::value, 2)};};

float length(cv::Point2f const& p);

float angle(cv::Point2f const&);

bool operator <(Pointf const &, Pointf const&);

bool operator >(const Pointf&,  const Pointf&);

typedef std::set<Pointf> CoordinateContainer;

struct CompareY{
    bool operator() ( cv::Point2f a, cv::Point2f b ){ //
        return a.y <=b.y;
	}
}; 

struct CompareX{
	bool operator()(cv::Point2f a, cv::Point2f b){
		return a.x<=b.x;
	}
};



b2Vec2 getb2Vec2(cv::Point2f );

Pointf getPointf(b2Vec2);

Pointf Polar2f(float, float);

template <typename T>
std::vector<T> set2vec(std::set<T>);

template <typename T>
std::set<T> vec2set(std::vector<T> vec){
	std::set <T> set;
    for (T t:vec){
        set.emplace(t);
    }
    return set;
}


class PointCloudProc{
	friend Configurator;
    std::vector <Pointf> previous;
    public:
    PointCloudProc(){};

    b2Transform affineTransEstimate(std::vector <Pointf>, Task::Action, float timeElapsed=0.2, float range=1.0);

	std::vector<Pointf> neighbours(b2Vec2,float radius =0.025); //finds if there are bodies close to a point. Used for 

	std::pair <bool, float>  findOrientation(std::vector<Pointf> ); //finds  average slope of line passign through two points in a radius of 2.5 cm. Assumes low clutter 

};

class ImgProc{
	std::vector <cv::Point2f> previousCorners;
    public:
    ImgProc(){}

    cv::Mat cropLeft(cv::Mat);

    cv::Mat cropRight(cv::Mat);

    void opticFlow(const cv::Mat&);

    private:
	int it=0;
	struct GoodFeaturesParameters{
		const int MAX_CORNERS=100;
    	const float QUALITY_LEVEL=0.5;
   		const int MIN_DISTANCE=7;
    	const int BLOCK_SIZE=7;
	}gfp;

    std::vector <cv::Point2f> corners; //must be single-precision float
    cv::Mat previousFrame_grey;
};



#endif