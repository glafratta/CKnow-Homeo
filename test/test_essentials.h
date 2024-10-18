#ifndef TEST_ESSENTIALS_H
#define TEST_ESSENTIALS_H
#include "configurator.h"
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip> 
#include <sstream> //for writing string into file, for checking, std::ostringstream
#include <ncurses.h>
#include <ctime>
#include <dirent.h>
#include <filesystem>
#define _USE_MATH_DEFINES


void Configurator::done_that(vertexDescriptor& src, bool & plan_works, b2World & world, std::vector<vertexDescriptor>& plan_provisional){
		//std::pair<bool, vertexDescriptor> been(false, TransitionSystem::null_vertex());
		//was ve instead of src
		std::vector <vertexDescriptor> options_src;
        //IF THE CURRENT PLAN HAS EXECUTED
		//if (bool fin=controlGoal.checkEnded(transitionSystem[src], UNDEFINED, true).ended; fin ){ //&& currentVertex!=movingVertex
		//printf("is target=%i, task ended = %i\n", target.getAffIndex()==PURSUE, fin);
			std::vector <BodyFeatures> b_features=worldBuilder.getFeatures(data2fp, b2Transform(b2Vec2(0,0), b2Rot(0)), currentTask.direction, BOX2DRANGE);
			//Disturbance where=controlGoal.disturbance;
			if (!b_features.empty()){
				State s_temp;
				s_temp.Dn= Disturbance(b_features[0]); //assumes 1 item length
				bool closest_match=1;
				findMatch(s_temp,transitionSystem, transitionSystem[movingEdge.m_source].ID, UNDEFINED, StateMatcher::DISTURBANCE, &options_src);
				//FIND STATE WHICH matches the relationship with the disturbance
			}
			//been= been_there(transitionSystem, where); 
		//}
		//printf("checked been = %i\n", been.first);

		//std::vector <vertexDescriptor> plan_provisional=planVertices;
	//	if (been.first){
		//	printf("provisional plan\n");
		for (auto o:options_src){
			plan_provisional=planner(transitionSystem, o); //been.second, been.first
			vertexDescriptor end =*(plan_provisional.rbegin().base()-1);
			if (controlGoal.checkEnded(transitionSystem[end]).ended && checkPlan(world, plan_provisional, transitionSystem)){
				plan_works=true;
				break;
			}
		}

}

std::vector <BodyFeatures> WorldBuilder::processData(const CoordinateContainer& points, const b2Transform& start){
    std::vector <BodyFeatures> result;
    std::vector <Pointf> ptset= set2vec(points);
    std::pair<bool,BodyFeatures> feature= getOneFeature(ptset);
    if (feature.first){
        feature.second.pose.q.Set(start.q.GetAngle());
        result.push_back(feature.second);
    }
    return result;
}


#endif