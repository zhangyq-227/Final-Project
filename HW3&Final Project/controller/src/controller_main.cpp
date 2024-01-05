

#include <iostream>

#include "controller_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reference_line");
    carla_pnc::ControllerNode controller_node;
    ROS_INFO("The controller is starting to operate");
    controller_node.MainLoop();
    return 0;
}