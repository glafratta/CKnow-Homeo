#include "configurator.h"

std::vector <BodyFeatures> WorldBuilder::processData(CoordinateContainer c){

}


int desired_split_size(b2Vec2 pos, float simulationStep){
    return int(pos.Length()/(simulationStep+0.00001))+1;
}

int main(int argc, char** argv){
    if (argc<4){
        throw std::invalid_argument("too few arguments\n");
    }

    b2Vec2 pos(atof(argv[1]), atof(argv[2]));
    b2Rot rot(atof(argv[3]));
    b2Transform pos2, pos3;
    Task goal=Task();
    Configurator conf(goal);
    conf.applyAffineTrans(b2Transform(b2Vec2(0,0),b2Rot(M_PI_2)), pos2);
    conf.applyAffineTrans(b2Transform(b2Vec2(0,0),b2Rot(-M_PI_2)), pos3);
    conf.simulationStep=std::max(ROBOT_HALFLENGTH, ROBOT_HALFWIDTH)*2;
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    b2Transform start=conf.transitionSystem[conf.movingVertex].endPose;
    if (argc>=7){
        start.p.x=atof(argv[4]);
        start.p.y=atof(argv[5]);
        start.q.Set(atof(argv[6]));
    }
    else{
        start.q.Set(rot.GetAngle());
    }
    //vertex simulating straight task (cul de sac)
    auto v1 = boost::add_vertex(conf.transitionSystem);
    auto e1 = boost::add_edge(conf.movingVertex, v1, conf.transitionSystem);
    conf.transitionSystem[e1.first].direction=DEFAULT;
    conf.transitionSystem[v1].outcome=simResult::crashed;    
    conf.transitionSystem[v1].endPose=b2Transform(pos, rot);
    conf.transitionSystem[v1].disturbance=Disturbance(AVOID,  b2Vec2(0.68, 0), 0);
    //simulating left
    auto v2 = boost::add_vertex(conf.transitionSystem);
    auto e2 = boost::add_edge(conf.movingVertex, v2, conf.transitionSystem);
    conf.transitionSystem[e2.first].direction=DEFAULT;
    conf.transitionSystem[v2].endPose=pos2;
    //simulating right
    auto v3 = boost::add_vertex(conf.transitionSystem);
    auto e3 = boost::add_edge(conf.movingVertex, v3, conf.transitionSystem);
    conf.transitionSystem[e3.first].direction=DEFAULT;
    conf.transitionSystem[v3].endPose=pos3;
    //add to ev queue
    std::vector <vertexDescriptor> evaluationQ={v1, v2, v3}, priorityQ, closed;
    //backtrack
    conf.backtrack(evaluationQ, priorityQ, closed, conf.transitionSystem);
    boost::print_graph(conf.transitionSystem);
    auto vs=boost::vertices(conf.transitionSystem);
    for (auto vi=vs.first; vi!=vs.second;vi++){
        if (!conf.transitionSystem[*vi].visited()& *vi!=conf.movingVertex ){
            throw std::logic_error("not visited");
        }
    }
    if (priorityQ[0]==v1){
        throw std::logic_error("collided is best next");
    }
    return 0;
}