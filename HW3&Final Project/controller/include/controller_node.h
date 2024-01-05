

#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H
#include <cmath>
#include <iostream>
#include <math.h>
#include <string>

// ros
#include "ros/ros.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"

// msg
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "controller.h"

#include "waypoint_msgs/Waypoint.h"
#include "waypoint_msgs/WaypointArray.h"

#include <Eigen/Eigen>

#include "controller.h"

namespace carla_pnc
{
    class ControllerNode
    {
    public:
        /***********************************Vehicle Parameters**************************************/
        double L = 3.0;                                                       // Wheelbase
        double cf = -155494.663;                                                // Front wheel cornering stiffness, sum of left and right
        double cr = -155494.663;                                                // Rear wheel cornering stiffness, sum of left and right
        double mass_fl = 1845.0 / 4;                                            // Mass of left front suspension
        double mass_fr = 1845.0 / 4;                                            // Mass of right front suspension
        double mass_rl = 1845.0 / 4;                                            // Mass of left rear suspension
        double mass_rr = 1845.0 / 4;                                            // Mass of right rear suspension
        double mass_front = mass_fl + mass_fr;                                  // Front suspension mass
        double mass_rear = mass_rl + mass_rr;                                   // Rear suspension mass
        double mass = mass_front + mass_rear;                                   // Vehicle load
        double lf = L * (1.0 - mass_front / mass);                              // Distance from the car's front wheel to the center point
        double lr = L * (1.0 - mass_rear / mass);                               // Distance from the car's rear wheel to the center point
        double Iz = std::pow(lf, 2) * mass_front + std::pow(lr, 2) * mass_rear; // Moment of inertia for vehicle rotation about the z-axis

        ControllerNode();
        void MainLoop();


    
    protected:
        std::string role_name;
        car_state cur_pose;                     // Current state of the vehicle
        std::vector<car_state> local_waypoints; // Local planning waypoints
        nav_msgs::Path path_msg;                // Driving path information
        carla_pnc::PIDController PidControll;    //PID Controller structure

        /***********************************Sbuscriber**************************************/
        ros::Subscriber cur_pose_sub;   // Subscribe to `/carla/<ROLE NAME>/odometry` to get the current state of the vehicle
        ros::Subscriber local_path_sub; // Subscribe to `/reference_line/local_waypoint` to get local planning trajectory points

        /***********************************Publisher**************************************/
        ros::Publisher path_pub;        // Publish vehicle driving path information
        ros::Publisher control_cmd_pub; // Publish control commands


        /***********************************callback**************************************/
        void callbackCarlaOdom(const nav_msgs::Odometry::ConstPtr &msg);
        void callbackLocalPath(const waypoint_msgs::WaypointArray::ConstPtr &msg);
    };

} // carla_pnc

#endif // CONTROLLER_NODE_H
