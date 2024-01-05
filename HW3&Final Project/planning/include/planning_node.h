
#ifndef PLANNING_NODE_H
#define PLANNING_NODE_H
#include <iostream>
#include <sstream>
#include <iomanip>
#include <unordered_map>
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <derived_object_msgs/Object.h>
#include <derived_object_msgs/ObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>

#include <waypoint_msgs/Waypoint.h>
#include <waypoint_msgs/WaypointArray.h>

#include <memory>
#include "collision_detection/collision_detection.h"
#include "reference_line/reference_line.h"
#include "reference_line/cubic_spline.hpp"
#include "point_types.h"
#include "common.h"


// #include "Optimal_Frenet_Planner/Optimal_Frenet_Planner.h"

namespace carla_pnc
{
    class PlanningNode
    {
    public:
        PlanningNode();
        void MainLoop();

    protected:
        std::vector<path_point> global_path; // Global path
        car_state cur_pose;                  // Current vehicle state
        bool planner_activate;               // Flag to activate the planner
        double path_length;                  // Length of the local planning reference line
        double cruise_speed;                 // Set cruise speed in m/s
        bool first_loop;                     // Flag to determine if it is the first loop
        int pre_match_index;                 // Previous cycle's matching point index
        bool get_odom;                       // Flag to indicate whether localization information is received
        bool car_follow;                     // Flag to indicate whether the vehicle is following another vehicle

        std::vector<Obstacle> detected_objects; // Detected obstacles

        std::vector<path_point> local_path; // Reference line extracted from the global path (before smoothing)
        std::vector<path_point> ref_path;   // Smoothed reference line

        FrenetPath final_path;     // Optimal planning trajectory
        FrenetPath pre_final_path; // Planning trajectory from the previous cycle

        std::vector<FrenetPath> sample_paths;  // For visualizing all sampled trajectories
        std::vector<FrenetPath> history_paths; // For visualizing historical sampled trajectories

        /***********************************Params**************************************/
        std::string role_name;
        std::string planning_method;
        // collision_detection params
        double collision_distance;

        std::unordered_map<std::string, double> referline_params;

        /***********************************Subscriber**************************************/

        ros::Subscriber cur_pose_sub;         
        ros::Subscriber global_path_sub;      
        ros::Subscriber cruise_speed_sub;     
        ros::Subscriber imu_sub;              
        ros::Subscriber detected_objects_sub; 

        /***********************************Publisher**************************************/

        ros::Publisher local_waypoints_pub; // Publish local path point information to the controller
        ros::Publisher ref_path_pub;        
        ros::Publisher sample_paths_pub;    // Publish sampled trajectories for RViz visualization
        ros::Publisher final_path_pub;     
        ros::Publisher history_paths_pub;   // Publish historical reference paths for RViz visualization

        ros::Publisher speed_marker_pub; 
        /***********************************callback**************************************/

        void callbackCarlaOdom(const nav_msgs::Odometry::ConstPtr &msg);

        void callbackCruiseSpeed(const std_msgs::Float32::ConstPtr &msg);

        void callbackGlobalPath(const nav_msgs::Path::ConstPtr &msg);

        void callbackIMU(const sensor_msgs::Imu::ConstPtr &msg);

        void callbackDetectedObjects(const derived_object_msgs::ObjectArray::ConstPtr &msg);

        /*********************************visualization****************************************/
        void ref_path_visualization(const std::vector<path_point> &ref_path);

        void final_path_visualization(const FrenetPath &final_path);

        void sample_paths_visualization(const std::vector<FrenetPath> &sample_paths);

        void history_path_visualization(const std::vector<FrenetPath> &histroy_paths);

        void object_speed_visualization(const std::vector<Obstacle> &detected_objects);
    };

} // namespace carla_pnc
#endif // PLANNING_NODE_H