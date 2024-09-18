#include "custom.h"

void forget(Configurator *c){}

void Configurator::done_that(vertexDescriptor& src, bool & plan_works, b2World & world, std::vector<vertexDescriptor>& plan_provisional){
		//std::pair<bool, vertexDescriptor> been(false, TransitionSystem::null_vertex());
		//was ve instead of src
		std::vector <vertexDescriptor> options_src;
		bool fin=controlGoal.checkEnded(transitionSystem[src], UNDEFINED, true).ended;
		printf("finished=%i, plan=%i, step=%i\n", fin, !planVertices.empty(), currentTask.motorStep);
		if ( !fin && planVertices.empty()&& currentTask.motorStep==0){
		//printf("is target=%i, task ended = %i\n", target.getAffIndex()==PURSUE, fin);
			printf("NOW CHECKING FOR MATCHES\n");
			bool using_kmeans=1;
			std::vector <BodyFeatures> b_features=worldBuilder.getFeatures(data2fp, transitionSystem[movingVertex].endPose, currentTask.direction, BOX2DRANGE, 0.15, using_kmeans);
			//Disturbance where=controlGoal.disturbance;
			printf("built world, b_feaures=%i\n", b_features.size());
			if (b_features.size()>0){
				printf("there is disturbance\n");
				State s_temp;
				WorldBuilder::CompareCluster compareCluster;
				BodyFeatures closest_D= std::min(b_features.begin(), b_features.end, compareCluster);
				s_temp.disturbance= Disturbance(closest_D); //assumes 1 item length
				debug::print_pose(s_temp.disturbance.pose());
				printf("half w=%f, half length=%f\n", s_temp.disturbance.bf.halfWidth, s_temp.disturbance.bf.halfLength);
				if (transitionSystem.m_vertices.size()>3){
					printf("v2 hw=%f, hl=%f, ", transitionSystem[2].disturbance.bf.halfWidth, transitionSystem[2].disturbance.bf.halfLength);
					debug::print_pose(transitionSystem[2].endPose);
				}
				bool closest_match=1;
				findMatch(s_temp,transitionSystem, NULL, UNDEFINED, StateMatcher::DISTURBANCE, &options_src);
				printf("looked for matches\n");
				//FIND STATE WHICH matches the relationship with the disturbance
			}
			else{
				vertexDescriptor new_v;
				auto e=addVertex(currentVertex, new_v, transitionSystem, Disturbance());
				simResult fake_result;
				fake_result.endPose=b2Transform(b2Vec2(BOX2DRANGE, 0), b2Rot(0));
				fake_result.step=100;
				printf("made fake result\n");
				gt::fill(fake_result, transitionSystem[new_v].ID);
				printf("filled\n");
				transitionSystem[e.first].direction=DEFAULT;
				transitionSystem[e.first].it_observed=iteration;
				//transitionSystem[e.first].step=100;
				plan_provisional={new_v};
				plan_works=true;
			}
			//been= been_there(transitionSystem, where); 
		}
		else{
			plan_works=true;//temporary to simplify things
		}
		//printf("checked been = %i\n", been.first);
		//std::vector <std::pair <vertexDescriptor, vertexDescriptor>> toRemove;

		//std::vector <vertexDescriptor> plan_provisional=planVertices;
	//	if (been.first){
		//	printf("provisional plan\n");
		printf("options=%i\n", options_src.size());
		for (auto o:options_src){
			printf("FOUND MATCH WITH %i!", int(o));
			plan_provisional=planner(transitionSystem, o); //been.second, been.first
			vertexDescriptor end =*(plan_provisional.rbegin().base()-1);
			if (controlGoal.checkEnded(transitionSystem[end]).ended && checkPlan(world, plan_provisional, transitionSystem)){
				src=plan_provisional[0];
				plan_works=true;
				return;
			}
			plan_provisional.clear();
		}

}


Disturbance set_target(int& run, b2Transform start){
	return Disturbance(PURSUE, b2Vec2(1.0f, 0.0f), 0.0f);

}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	configurator.planning =1;
	char name[60];
	//sprintf(name, "Dmatch_target");
	//get_Foldername(,name);
	//printf("foldername = %s\n", name);
	configurator.setBenchmarking(1, "Dmatch_target");
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
	}
	configurator.setSimulationStep(.27);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface);
	MotorCallback cb(&configurator);
	lidar.registerInterface(&dataInterface);
	CameraCallback cameraCB(&cb);
	Libcam2OpenCV camera;
    camera.registerCallback(&cameraCB);
    Libcam2OpenCVSettings settings;
    settings.framerate = FPS;
	motors.registerStepCallback(&cb);
	configurator.start();
	lidar.start();
    camera.start(settings);
	motors.start();
	getchar();
	configurator.stop();
	motors.stop();
	lidar.stop();
	camera.stop();
}
	
	
