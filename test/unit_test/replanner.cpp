#include "../callbacks.h"

int main(int argc, char** argv){
    bool debug=1;
    Disturbance target1;
    vertexDescriptor solution=TransitionSystem::null_vertex();
    if (argc>2){
        solution=vertexDescriptor(atoi(argv[2]));
    }
    else{
        throw;
    }
    if (argc>3){
        if (atoi(argv[3])==1){
            target1= Disturbance(PURSUE, b2Vec2(1.0,0), 0);    
            printf("target");
        }
    }
    Task goal(target1,DEFAULT);
    Configurator conf(goal);
    conf.simulationStep=0.27;
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    conf.setBenchmarking(true, "simulation_benchmarking");
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
    conf.planVertices= conf.planner(conf.transitionSystem, conf.currentVertex);
    printf("OG PLAN: ");
    int n_v=conf.transitionSystem.m_vertices.size();
    conf.printPlan(&conf.planVertices);
    conf.addIteration();
    conf.currentVertex=*(conf.planVertices.end()-1);
    vertexDescriptor prev=*(conf.planVertices.end()-2);
    conf.currentEdge=boost::edge(prev, conf.currentVertex, conf.transitionSystem).first;
    std::vector <vertexDescriptor> options_src;
    State state_tmp;
    b2Transform shift= b2Transform(b2Vec2(1,0), b2Rot(0));
    conf.applyAffineTrans(shift, conf.transitionSystem);    
    if (argc>4){
        di.iteration=atoi(argv[4]);
        di.newScanAvail();          
        conf.data2fp = ci.data2fp;
    }
    std::vector <BodyFeatures> b_features=conf.worldBuilder.getFeatures(conf.data2fp, state_tmp.start, DEFAULT, BOX2DRANGE);
    if (!b_features.empty()){
        state_tmp.Dn= Disturbance(b_features[0]); //assumes 1 item length
    }
    bool relax_match=1, plan_works=false;
    boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    conf.findMatch(state_tmp,conf.transitionSystem, NULL, UNDEFINED, StateMatcher::DISTURBANCE, &options_src, relax_match);

    std::vector<vertexDescriptor> plan_provisional;
    auto been=conf.been_there(conf.transitionSystem, conf.controlGoal.disturbance);
    for (vertexDescriptor o: options_src){
        conf.recall_plan_from(o, conf.transitionSystem, world, plan_provisional, plan_works, been);
    }
    if (conf.transitionSystem.m_vertices.size() > n_v){
        printf("size error = %i\n", conf.transitionSystem.m_vertices.size());
        return 2;
    }
    if (plan_provisional!=conf.planVertices){
        printf("PROVISIONAL: ");
        conf.printPlan(&plan_provisional);

        return 1;
    }
     
    return 0;
}