#include "simulator/simulation_wrapper.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "simulation");
    SimulationWrapper sim;


    sim.init();

    ros::spinOnce();


    ros::Rate r(50);
    while (ros::ok() && sim.spinOnes()) {
        ros::spinOnce();
        r.sleep();
    }

    
    

}
