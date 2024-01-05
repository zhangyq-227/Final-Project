
#include "controller_node.h"
#include "controller.h"

using namespace std;

namespace carla_pnc
{
  ControllerNode::ControllerNode()
  {
    ros::NodeHandle n("~"); 
    n.param<string>("role_name", role_name, "ego_vehicle");
    
    // setup subscriber
    cur_pose_sub = n.subscribe("/carla/" + role_name + "/odometry", 10, &ControllerNode::callbackCarlaOdom, this);
    local_path_sub = n.subscribe("/reference_line/local_waypoint", 10, &ControllerNode::callbackLocalPath, this);

    // setup publishers
    path_pub = n.advertise<nav_msgs::Path>("/trajectory", 10);
    // ref_pub = n.advertise<nav_msgs::Path>("/ref_trajectory", 10);
    control_cmd_pub = n.advertise<carla_msgs::CarlaEgoVehicleControl>(
        "/carla/" + role_name + "/vehicle_control_cmd", 10);
    
    //Initialize PID parameter
    InitialisePID(PidControll,1.0f,0.1,0.0f,2.0f,0.0f,0.1f,1.0f,0.0f,0.0f);
  }

    

  /**
   * @brief MainLoop
   *
   */

  // Your code for MainLoop() update 
  
  

  void ControllerNode::MainLoop()
  {

    std::vector<double> prev_p(2);
    double frequency = 50.0;
    ros::Rate rate(frequency);
    vector<double> cmd(3,0.0f);
    vector<double> pre_cmd(3);

    double last_timestamp = ros::Time::now().toSec();

    while (ros::ok())
    {
      // ROS_INFO("start Inteartion");
      double current_timestamp = ros::Time::now().toSec();

      double duration=current_timestamp-last_timestamp;

      if(duration < 0.0f || duration > 0.5f){
        duration = 0.05f;
      }

      last_timestamp=current_timestamp;
      
      double dist = hypot(cur_pose.x - prev_p[0], cur_pose.y - prev_p[1]);

      //差得太远,控制算法难以调整，交给planning重新规划路线
      if (dist > 10.0)
      {
        local_waypoints.clear();
        ROS_INFO("Restarting ego car");
      }

      prev_p[0] = cur_pose.x;
      prev_p[1] = cur_pose.y;


      // Use your controller
      /***********************************Depend on your controller**************************************/

      //输入：给定的当前位置以及planning模块规划出来的整条轨线
      //输出：控制命令

      /*
        CARLA中的指令格式：
        "# 0. <= throttle <= 1.\n"
        "float32 throttle\n"
        "\n"
        "# -1. <= steer <= 1.\n"
        "float32 steer\n"
        "\n"
        "# 0. <= brake <= 1.\n"
        "float32 brake\n"
        "\n"
        "# hand_brake 0 or 1\n"
        "bool hand_brake\n"
        "\n"
        "# reverse 0 or 1\n"
        "bool reverse\n"
      */
      
      //首先，在planning规划出来的轨迹当中找到目标点
      //选择欧式距离最近的点作为目标点
      //采用PID控制器进行控制
      if(local_waypoints.size() > 0){
        car_state Target = FindNearestPoint(cur_pose,local_waypoints);
        cmd = GetControlCmd(PidControll,cur_pose,Target,duration);
      }

      /***********************************Publishing Control Commands**************************************/
      carla_msgs::CarlaEgoVehicleControl control_cmd;
      control_cmd.throttle = cmd[0];
      control_cmd.steer = cmd[1];  
      control_cmd.brake = cmd[2];
      control_cmd.hand_brake = false;
      control_cmd.reverse = false;

      control_cmd_pub.publish(control_cmd);

      ros::spinOnce();
      rate.sleep();
      // ROS_INFO("The iteration end.");

    }
  }

 

  void ControllerNode::callbackCarlaOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
    geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_quat, quat);

    
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    cur_pose.x = msg->pose.pose.position.x;
    cur_pose.y = msg->pose.pose.position.y;
    cur_pose.yaw = yaw;

    cur_pose.vx = msg->twist.twist.linear.x;
    cur_pose.vy = msg->twist.twist.linear.y;
    cur_pose.v = std::sqrt(cur_pose.vx * cur_pose.vx + cur_pose.vy * cur_pose.vy);

    cur_pose.z = msg->pose.pose.position.z;

    
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
    this_pose_stamped.pose.position.y = msg->pose.pose.position.y;
    this_pose_stamped.pose.position.z = msg->pose.pose.position.z;

    this_pose_stamped.pose.orientation.x = msg->pose.pose.orientation.x;
    this_pose_stamped.pose.orientation.y = msg->pose.pose.orientation.y;
    this_pose_stamped.pose.orientation.z = msg->pose.pose.orientation.z;
    this_pose_stamped.pose.orientation.w = msg->pose.pose.orientation.w;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "map";
    path_msg.poses.push_back(this_pose_stamped);

    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    path_pub.publish(path_msg);
  }

  /**
   * @brief Get Local Path
   *
   * @param msg
   */
  void ControllerNode::callbackLocalPath(const waypoint_msgs::WaypointArray::ConstPtr &msg)
  {
   //ROS_INFO("Received final waypoints in trajectory controller ...");
    local_waypoints.resize(msg->waypoints.size());
    for (int i = 0; i < msg->waypoints.size(); i++)
    {
      car_state temp_point;
      temp_point.x = msg->waypoints[i].pose.pose.position.x;
      temp_point.y = msg->waypoints[i].pose.pose.position.y;
      temp_point.v = msg->waypoints[i].twist.twist.linear.x;
      local_waypoints[i] = temp_point;
    }
  }

} // carla_pnc