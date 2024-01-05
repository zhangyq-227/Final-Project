#include "../include/planning_node.h"
#include "Planner/EasyPlanner.h"
using namespace std;

namespace carla_pnc
{
    PlanningNode::PlanningNode()
    {
        cruise_speed = 5.0;
        ros::NodeHandle n("~"); // Node handle

        n.param<string>("role_name", role_name, "ego_vehicle");
        n.param<double>("path_length", path_length, 200.0);
        n.param<bool>("planner_activate", planner_activate, true);
        // n.param<bool>("planner_activate", planner_activate, false);

        n.param<string>("planning_method", planning_method, "Optimal_Frenet_Planner");

        // collision_detection params
        n.param<double>("collision_distance", collision_distance, 2.0);

        // setup subscriber
        cur_pose_sub = n.subscribe("/carla/" + role_name + "/odometry", 10, &PlanningNode::callbackCarlaOdom, this);
        global_path_sub = n.subscribe("/carla/" + role_name + "/waypoints", 10, &PlanningNode::callbackGlobalPath, this);
        cruise_speed_sub = n.subscribe("/cruise_speed", 10, &PlanningNode::callbackCruiseSpeed, this);
        imu_sub = n.subscribe("/carla/" + role_name + "/imu", 10, &PlanningNode::callbackIMU, this);
        detected_objects_sub = n.subscribe("/carla/" + role_name + "/objects", 10, &PlanningNode::callbackDetectedObjects, this);

        // setup publishers
        local_waypoints_pub = n.advertise<waypoint_msgs::WaypointArray>("/reference_line/local_waypoint", 10);
        ref_path_pub = n.advertise<nav_msgs::Path>("/reference_line/ref_path", 10);
        sample_paths_pub = n.advertise<nav_msgs::Path>("/reference_line/sample_paths", 10);
        final_path_pub = n.advertise<nav_msgs::Path>("/reference_line/final_path", 10);
        history_paths_pub = n.advertise<nav_msgs::Path>("/reference_line/history_paths", 10);
        speed_marker_pub = n.advertise<visualization_msgs::Marker>("/speed_marker_text", 10);

        pre_match_index = 0;
        get_odom = false;
        detected_objects.clear();
        car_follow = false;
        first_loop = false;

    }

    /**
     * @brief Callback function for Carla odometry information. Retrieves the current position of the vehicle based on odometry data.
     *
     * @param msg  nav_msgs/Odometry
     */
    void PlanningNode::callbackCarlaOdom(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Coordinate transformation
        geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(odom_quat, quat);

        // Retrieve roll, pitch, and yaw based on the transformed quaternion
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        cur_pose.x = msg->pose.pose.position.x;
        cur_pose.y = msg->pose.pose.position.y;
        cur_pose.z = msg->pose.pose.position.z;
        cur_pose.yaw = yaw;

        cur_pose.vx = msg->twist.twist.linear.x;
        cur_pose.vy = msg->twist.twist.linear.y;
        cur_pose.v = sqrt(pow(cur_pose.vx, 2) + pow(cur_pose.vy, 2));

        get_odom = true;
        // ROS_INFO("Current pose: x: %.2f, y: %.2f, yaw: %.2f, x_speed:%.2f,y_speed:%.2f,v_speed:%.2f",
        //          cur_pose.x, cur_pose.y, cur_pose.yaw, cur_pose.vx, cur_pose.vy, cur_pose.v);
    }

    void PlanningNode::callbackCruiseSpeed(const std_msgs::Float32::ConstPtr &msg)
    {
        cruise_speed = msg->data;
    }

    /**
     * @brief Process the received global path from Carla.
     *
     * @param msg Global path from Carla.
     */
    void PlanningNode::callbackGlobalPath(const nav_msgs::Path::ConstPtr &msg)
    {
        global_path.clear();
        path_point global_path_point;

        // Convert path information to global_path
        for (int i = 0; i < msg->poses.size(); i++)
        {
            global_path_point.x = msg->poses[i].pose.position.x;
            global_path_point.y = msg->poses[i].pose.position.y;
            global_path_point.z = msg->poses[i].pose.position.z;

            // Coordinate transformation
            geometry_msgs::Quaternion odom_quat = msg->poses[i].pose.orientation;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(odom_quat, quat);

            // Get roll, pitch, yaw based on the converted quaternion
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            global_path_point.yaw = yaw;

            //如果当前的数组当中没有数据，那么直接将坐标点放进数组当中
            if (global_path.size() == 0)
            {
                global_path.push_back(global_path_point);
            }

            // Avoid overlapping path points
            //如果有数据的话，判断和上一个数据点之间的距离，如果距离相差比较远的话再将其加入数组
            //这一步的目的是为了避免考虑重复的点
            else if ((global_path.size() > 0) &&
                     abs(global_path_point.x - global_path[global_path.size() - 1].x) >
                         0.001 &&
                     abs(global_path_point.y - global_path[global_path.size() - 1].y) >
                         0.001)
            {
                global_path.push_back(global_path_point);
                // ROS_INFO("global_x:%.f,global_y:%.f",path_point.x,path_point.y);
            }
        }
        // ROS_INFO("Received global_path successfully, The size of global is :%zu", global_path.size());
    }

    /**
     * @brief Get IMU information (calculate acceleration)
     *
     * @param msg
     */
    void PlanningNode::callbackIMU(const sensor_msgs::Imu::ConstPtr &msg)
    {

        cur_pose.ax = msg->linear_acceleration.x;
        cur_pose.ay = msg->linear_acceleration.y;
        cur_pose.a = sqrt(cur_pose.ax * cur_pose.ax +
                          cur_pose.ay * cur_pose.ay); // Acceleration
    }

    /**
     * @brief Get information about detected objects
     * https://docs.ros.org/en/melodic/api/derived_object_msgs/html/msg/ObjectArray.html
     * @param msg
     */
    //获取其他物体的位置和姿态,检测到其他的障碍物
    void PlanningNode::callbackDetectedObjects(
        const derived_object_msgs::ObjectArray::ConstPtr &msg)
    {
        // ROS_INFO("Received dectected objects successfully ...");

        // Convert object data
        detected_objects.resize(msg->objects.size());
        // cout<<"obstacle sizes: "<<msg->objects.size()<<endl;
        for (int i = 0; i < msg->objects.size(); i++)
        {
            Obstacle ob;

            ob.point.x = msg->objects[i].pose.position.x;
            ob.point.y = msg->objects[i].pose.position.y;
            ob.point.z = msg->objects[i].pose.position.z;
            ob.point.vx = msg->objects[i].twist.linear.x;
            ob.point.vy = msg->objects[i].twist.linear.y;
            ob.point.v = sqrt(pow(ob.point.vx, 2) + pow(ob.point.vy, 2));
            // ROS_INFO("leader car speedvx: %.2f , position,vy:%.2f,v:%.2f",ob.point.vx,ob.point.vy,ob.point.v);
            
            // Coordinate transformation
            geometry_msgs::Quaternion ob_quat = msg->objects[i].pose.orientation;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(ob_quat, quat);

            // Get roll, pitch, yaw based on the transformed quaternion
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            ob.point.yaw = yaw;

            ob.x_rad = msg->objects[i].shape.dimensions[0] / 2.0;
            ob.y_rad = msg->objects[i].shape.dimensions[1] / 2.0;
            double z_rad = 0.8;

            detected_objects[i] = ob;
        }
    }

    /**
     * @brief MainLoop
     *
     */
    void PlanningNode::MainLoop()
    {
        ros::Rate rate(10.0);

        /***********************************Path Planning**************************************/

        carla_pnc::ReferenceLine reference_line(path_length,
                                                referline_params);
        
        int start_index = pre_match_index;

        carla_pnc::Easy_Planner planner;


        while (ros::ok())
        {
            // ROS_INFO("w_object%.2f, w_lon_jerk%.2f,  w_lat_offset%.2f ,w_lat_acc%.2f", w_object, w_lon_jerk, w_lat_offset, w_lat_acc);
            // double current_timestamp = ros::Time::now().toSec();

            // Start planning after obtaining the localization information
            if (get_odom && !global_path.empty())
            {
                start_index = pre_match_index;

                /***********************************Step1 Get Local Planning Reference Line from Global Path**************************************/
                local_path = reference_line.local_path_truncation(cur_pose, global_path, start_index);
                
                // ROS_INFO("The pre_match_index is %d:", reference_line.match_index);
                // ROS_INFO("The size of Global path is %d:", global_path.size());
                
                pre_match_index = reference_line.match_index;
                // prev_timestamp = current_timestamp;

                if (local_path.size() > 1)
                {
                    /***********************************Step2 Smooth the Reference Line**************************************/

                    // Use Spline2D to construct a smooth Frenet coordinate system
                    std::vector<double> x_set, y_set;
                    for (int i = 0; i < local_path.size(); i++)
                    {
                        x_set.push_back(local_path[i].x);
                        y_set.push_back(local_path[i].y);
                    }

                    Spline2D ref_frenet(x_set, y_set);
                    
                    // Cubic Spline smoothing
                    // ROS_INFO("curblic Spline smooth");
                    ref_path = reference_line.smoothing(ref_frenet, local_path);
                    
                    /***********************************Step3 Confirm the planning starting point and project it into the Frenet coordinate system to obtain (s0, l0).**************************************/
                    if (planner_activate)
                    {
                        car_state global_initial_point; // Planning starting point in the global coordinate system

                        // If it is the first run or the planned trajectory from the previous cycle does not exist,then use the current position of the vehicle as the starting point for this cycle's planning.
                        if (first_loop || pre_final_path.frenet_path.size() < 5 )
                        {
                            global_initial_point = cur_pose;
                            first_loop = false;
                        }

                        // /*
                        //  * Compare the current position of the vehicle with the matched trajectory point from the previous cycle based on the current_time.
                        //  * If the difference is too large, then calculate the position for the next cycle based on the current vehicle position using kinematic formulas, use it as the starting point for planning in this cycle.
                        //  */
                        // else if (fabs(cur_pose.x - pre_final_path.frenet_path[0].x) > 2.0 ||
                        //          fabs(cur_pose.y - pre_final_path.frenet_path[0].y) > 0.5)
                        // {
                        //     // Calculate the position for the next 100 milliseconds using kinematic formulas.
                        //     car_state next_pose;
                        //     double dt = 0.1;
                        //     next_pose = cur_pose;

                        //     // Convert the vehicle body velocity to global velocity.
                        //     double vx = cur_pose.vx * cos(cur_pose.yaw) - cur_pose.vy * sin(cur_pose.yaw);
                        //     double vy = cur_pose.vx * sin(cur_pose.yaw) + cur_pose.vy * cos(cur_pose.yaw);

                        //     double ax = cur_pose.ax * cos(cur_pose.yaw) - cur_pose.ay * sin(cur_pose.yaw);
                        //     double ay = cur_pose.ay * sin(cur_pose.yaw) + cur_pose.ay * cos(cur_pose.yaw);

                        //     next_pose.x = cur_pose.x + vx * dt + 0.5 * ax * dt * dt;
                        //     next_pose.y = cur_pose.y + vy * dt + 0.5 * ay * dt * dt;

                        //     next_pose.vx = cur_pose.vx + cur_pose.ax * dt;
                        //     next_pose.vy = cur_pose.vy + cur_pose.ay * dt;
                        //     next_pose.v = sqrt(pow(next_pose.vx, 2) + pow(next_pose.vy, 2));

                        //     global_initial_point = next_pose;
                        // }
                        // // If the difference is not significant, select the trajectory point at current_time + 100ms as the planning starting point for this cycle.
                        // else if (pre_final_path.frenet_path.size())
                        // {
                        //     if (planning_method == "Optimal_Frenet_Planner")
                        //     {
                        //         global_initial_point = pre_final_path.frenet_path[5];
                        //     }
                        
                        // }

                        // to frenet state
                        //
                        // FrenetPoint frenet_initial_point = calc_frenet(global_initial_point, ref_path);
                        // // ROS_INFO("Get Initial Point Successfully");

                        // /***********************************Step4 Use your planner **************************************/

                        CollisionDetection collision_detection(detected_objects, collision_distance, ref_path); // collision detection

                        // Your code for update : Use your planner
                        //To Do: planner algorithm
                        
                        if(ref_path.size() > 0){
                            // carla_pnc::Easy_Planner planner1;
                            planner.set_state_loop(global_initial_point,detected_objects,collision_distance,ref_path);
                            this->final_path = planner.run();
                            // cout<<"final size:"<<final_path.frenet_path.size()<<endl;
                        }
                        
                        
                        /*******************************visualization******************************************/
                        // Optimal planning trajectory for Rviz visualization
                        final_path_visualization(final_path);

                        // Display the speed of obstacles
                        object_speed_visualization(collision_detection.detected_objects);
                    }
                }
            }
            // Visualization of the reference line
            ref_path_visualization(ref_path);

            // Historical planning trajectory for Rviz visualization
            history_path_visualization(history_paths);

            /***********************************Publish**************************************/
            // Publish planned path points to the controller
            waypoint_msgs::WaypointArray local_waypoints;
            local_waypoints.header.frame_id = "map";

            if (planner_activate)
            {
                // ROS_INFO("planner activates");
                for (int i = 0; i < final_path.frenet_path.size(); i++)
                {
                    //只需要规划器能够得到x,y坐标和速度即可
                    waypoint_msgs::Waypoint point;
                    point.pose.pose.position.x = final_path.frenet_path[i].x;
                    point.pose.pose.position.y = final_path.frenet_path[i].y;
                    point.twist.twist.linear.x = final_path.frenet_path[i].v;
                    // std::cout<<final_path.frenet_path[i].v<<std::endl;
                    local_waypoints.waypoints.push_back(point);
                }
            }
            //未启动planning节点
            else
            {
                // ROS_INFO("planner unactivates");
                for (int i = 0; i < ref_path.size(); i++)
                {
                    waypoint_msgs::Waypoint point;
                    point.pose.pose.position.x = ref_path[i].x;
                    point.pose.pose.position.y = ref_path[i].y;
                    point.twist.twist.linear.x = cruise_speed;
                    local_waypoints.waypoints.push_back(point);
                }
            }
            // ROS_INFO("The size of local path is:%zu", ref_path.size());
            local_waypoints_pub.publish(local_waypoints);

            ros::spinOnce();
            rate.sleep();
            // ROS_INFO("The iteration end.");
        }
    }

    /***********************************visualization**************************************/

    /**
     * @brief 
     *
     * @param ref_path
     */
    void PlanningNode::ref_path_visualization(const std::vector<path_point> &ref_path)
    {
        nav_msgs::Path path;
        path.header.frame_id = "map";
        for (int i = 0; i < ref_path.size(); i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = ref_path[i].x;
            pose.pose.position.y = ref_path[i].y;


            geometry_msgs::Quaternion quat =
                tf::createQuaternionMsgFromYaw(ref_path[i].yaw);
            pose.pose.orientation = quat;
            path.poses.push_back(pose);
        }
        ref_path_pub.publish(path);
    }

    /**
     * @brief 
     *
     * @param final_path
     */
    void PlanningNode::final_path_visualization(const FrenetPath &final_path)
    {
        nav_msgs::Path best_path;
        best_path.header.frame_id = "map";
        for (int i = 0; i < final_path.frenet_path.size(); i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = final_path.frenet_path[i].x;
            pose.pose.position.y = final_path.frenet_path[i].y;


            geometry_msgs::Quaternion quat =
                tf::createQuaternionMsgFromYaw(final_path.frenet_path[i].yaw);
            pose.pose.orientation = quat;
            best_path.poses.push_back(pose);
        }
        final_path_pub.publish(best_path);
    }

    /**
     * @brief 
     *
     * @param sample_paths
     */
    void PlanningNode::sample_paths_visualization(const std::vector<FrenetPath> &sample_paths)
    {
        //
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        for (auto final_path : sample_paths)
        {
            for (int i = 0; i < final_path.size_; i += 10)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = final_path.frenet_path[i].x;
                pose.pose.position.y = final_path.frenet_path[i].y;
                // pose.pose.position.z = final_path.z[i];

                geometry_msgs::Quaternion quat =
                    tf::createQuaternionMsgFromYaw(final_path.frenet_path[i].yaw);
                pose.pose.orientation = quat;
                path.poses.push_back(pose);
            }
            sample_paths_pub.publish(path);
        }
    }

    /**
     * @brief 
     *
     * @param history_path
     */
    void PlanningNode::history_path_visualization(const std::vector<FrenetPath> &history_paths)
    {
        nav_msgs::Path history_path;
        history_path.header.frame_id = "map";
        history_path.header.stamp = ros::Time::now();
        for (auto final_path : history_paths)
        {
            int count = 0;
            for (int i = 0; i < final_path.size_ && count < 1; i++, count++)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = final_path.frenet_path[i].x;
                pose.pose.position.y = final_path.frenet_path[i].y;
                // pose.pose.position.z = final_path.z[i];

                geometry_msgs::Quaternion quat =
                    tf::createQuaternionMsgFromYaw(final_path.frenet_path[i].yaw);
                pose.pose.orientation = quat;
                history_path.poses.push_back(pose);
            }
            history_paths_pub.publish(history_path);
        }
    }

    /**
     * @brief 
     *
     * @param detected_objects
     */
    void PlanningNode::object_speed_visualization(const std::vector<Obstacle> &detected_objects)
    {
        // speed_marker_pub = n.advertise<visualization_msgs::Marker>("/speed_marker_text", 10);

        int id_ = 0;
        for (auto &object : detected_objects)
        {
            visualization_msgs::Marker speed_marker;
            speed_marker.header.frame_id = "map";
            speed_marker.header.stamp = ros::Time::now();
            speed_marker.ns = "planning/speed_marker";
            speed_marker.action = visualization_msgs::Marker::ADD;
            speed_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

            speed_marker.pose.orientation.w = 1.0;
            speed_marker.id = id_++;

            speed_marker.scale.x = 1.5;
            speed_marker.scale.y = 1.5;
            speed_marker.scale.z = 1.5;

            speed_marker.color.b = 0;
            speed_marker.color.g = 0;
            speed_marker.color.r = 255;
            speed_marker.color.a = 1;

            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << object.point.v; 
            speed_marker.text = ss.str() + "m/s";

            speed_marker.pose.position.x = object.point.x;
            speed_marker.pose.position.y = object.point.y;
            speed_marker.pose.position.z = object.point.z + 1;

            speed_marker_pub.publish(speed_marker);
        }
    }

} // carla_pnc
