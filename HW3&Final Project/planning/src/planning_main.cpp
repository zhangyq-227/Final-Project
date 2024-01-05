
#include <iostream>

#include "planning_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning");
    
    carla_pnc::PlanningNode planning_node;
    
    ROS_INFO("Start Planning");
    
    planning_node.MainLoop();

    return 0;
}