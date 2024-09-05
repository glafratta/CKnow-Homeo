#include "../callbacks.h"

std::vector <Direction> getPlan(const TransitionSystem & g, const std::vector <vertexDescriptor>& plan, vertexDescriptor pre){
    	std::vector <Direction>result;
        for (vertexDescriptor v: plan){
		std::pair <edgeDescriptor, bool> edge=boost::edge(pre, v, g);
		if (edge.second){
			// auto a=dirmap.find(g[edge.first].direction);
			// printf("%i, %s, ", edge.first.m_target, (*a).second);
            result.push_back(g[edge.first].direction);
		}
		pre=edge.first.m_target;
		}
        return result;

}


int main(int argc, char** argv){
    
    //we imagine that we have executed a plan and then the robot is instructed to go back on its steps
    bool debug=0;
    Disturbance target1;
    std::vector <Direction> solution={DEFAULT, DEFAULT}, solution2=solution;
    float simStep=0.5;
    if (argc>2){
        if (atoi(argv[2])==1){
            target1= Disturbance(PURSUE, b2Vec2(1.0,0), 0);  
            solution={DEFAULT, DEFAULT, LEFT, DEFAULT, RIGHT, DEFAULT, RIGHT, DEFAULT, LEFT, DEFAULT};  
            simStep=ROBOT_HALFLENGTH*2;
            solution2=solution;
        }
        else{
            solution={LEFT, DEFAULT, DEFAULT};
            solution2={RIGHT, DEFAULT, DEFAULT};
        }
    }
    Task goal(target1,DEFAULT);
    Configurator conf(goal);
    conf.setSimulationStep(simStep);
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    DataInterface di(&ci);
    if (argc>1){
        di.folder=argv[1];
        di.newScanAvail();          
    }
    conf.data2fp = ci.data2fp;
    conf.addIteration();
    b2World world(b2Vec2(0,0));
    boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    conf.dummy_vertex(conf.currentVertex);
    conf.explorer(conf.currentVertex, conf.transitionSystem, *conf.getTask(), world);
    std::vector <vertexDescriptor> plan=conf.planner(conf.transitionSystem, conf.currentVertex);
    std::vector <Direction> plan_d=getPlan(conf.transitionSystem, plan, conf.currentVertex);
    if (plan_d!=solution & plan_d !=solution2){
        conf.printPlan(&plan);
        return 1;
    }
    return 0;
}