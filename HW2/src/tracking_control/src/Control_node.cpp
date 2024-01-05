#include <ros/ros.h>
#include <iostream>
#include "Controller.h"
#include <vector>
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/Marker.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
using namespace std;

double freq,L,V_DESIRED;
double v_max;
bool limit_v_and_kesi;
double initial_x,initial_y,initial_yaw,initial_v,initial_kesi;
double slow_LEVE1_DISTANCE,slow_LEVE2_DISTANCE,slow_LEVE1_V,slow_LEVE2_V,goal_tolerance_DISTANCE;
#define pi acos(-1)
#define T 1/freq 

//如果使用PID设置为0，使用LQR则设置为1
#define USE_PID_OR_LQR 1
 
vehicleState update_state(U control, vehicleState car,double interval) {
	car.v = control.v;
	car.kesi = control.kesi;
	car.x += car.v * cos(car.yaw) * interval;
	car.y += car.v * sin(car.yaw) * interval;
	car.yaw += car.v / L * tan(car.kesi) * interval;
	return car;
}
 
 //定义一组路径，本质上由许多的点组成
class Path {
private:
	vector<waypoint> path;
public:

	void Add_new_point(waypoint& p)
	{
		path.push_back(p);
	}
 
	void Add_new_point(vector<waypoint>& p) 
	{
		path = p;
	}
 
	
	unsigned int Size()
	{
		return path.size();
	}
 

	waypoint Get_waypoint(int index)
	{
		waypoint p;
		p.ID = path[index].ID;
		p.x = path[index].x;
		p.y = path[index].y;
		p.yaw = path[index].yaw;
		return p;
	}

	vector<waypoint> Get_waypoints(){
		return path;
	}
 
 
	
	int Find_target_index(vehicleState state)
	{
		//并没有接收到另一个节点发送的消息
		// std::cout<< path.size()<< endl;
		if(!path.size())
		{
			return -1;
		}
		double min = abs(sqrt(pow(state.x - path[0].x, 2) + pow(state.y - path[0].y, 2)));
		int index = 0;
		for (int i = 0; i < path.size(); i++)
		{
			double d = abs(sqrt(pow(state.x - path[i].x, 2) + pow(state.y - path[i].y, 2)));
			if (d < min)
			{
				min = d;
				index = i;
			}
		}
 
	
		if ((index + 1) < path.size())
		{
			double current_x = path[index].x; double current_y = path[index].y;
			double next_x = path[index + 1].x; double next_y = path[index + 1].y;
			double L_ = abs(sqrt(pow(next_x - current_x, 2) + pow(next_y - current_y, 2)));
			double L_1 = abs(sqrt(pow(state.x - next_x, 2) + pow(state.y - next_y, 2)));
			
			if (L_1 < L_)
			{
				index += 1;
			}
		}
		return index;
	}
 
};

// depending on your controller
std::vector<double> Q_set;
std::vector<double> R_set;
 
class Control_node {
private:
	//car
	vehicleState car;
	U control;
	
	int lastIndex;
	waypoint lastPoint;
	string action;//(tracking or reach goal!)
	
	//ROS
	ros::Subscriber path_sub;
	ros::Publisher vel_pub;
	ros::Publisher actual_state_pub;
	ros::Publisher visual_state_pub;
	geometry_msgs::Point visual_state_pose;
	visualization_msgs::Marker visual_state_trajectory;
	geometry_msgs::Pose2D actual_pose;
	geometry_msgs::Twist vel_msg;
	int temp;

	//Timer
	ros::Time timer;
 
public:
    Controller* controller;
    Path* path;

	Control_node(ros::NodeHandle& nh)
	{
		//定义controller对象，实现控制算法
		controller = new Controller();
		//定义一条路径，在中断回调函数当中读取path_planning中发布的数据
		//通过这个类中的addpointcallback添加到path当中去
		path = new Path();
        
		//ROS:
		path_sub = nh.subscribe("path",10,&Control_node::addpointcallback,this);
		vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
		visual_state_pub = nh.advertise<visualization_msgs::Marker>("visualization_pose",10);
		actual_state_pub = nh.advertise<geometry_msgs::Pose2D>("Control_pose",10);

		//robot state initialize:
		car.x = initial_x;car.y = initial_y;car.yaw = initial_yaw;car.v = initial_v;car.kesi = initial_kesi;
		action = "the car is tracking!!";

		this->timer = ros::Time::now();
	}
 
	~Control_node() {
        		delete(controller);
        		delete(path);
	}
 
	void addpointcallback(const nav_msgs::Path::ConstPtr& msg){
		// std::cout<<"Path received!"<<endl;
		vector<waypoint> waypoints;
		for(int i=0;i<msg->poses.size();i++){
			waypoint waypoint;
			//ROS_INFO("THE PATH[%d]'s ID is %d",i,msg->poses[i].header.seq);
			waypoint.ID = msg->poses[i].header.seq;
			waypoint.x = msg->poses[i].pose.position.x;
			waypoint.y = msg->poses[i].pose.position.y;

			double roll,pitch,yaw;
	    		tf::Quaternion quat;
	    		tf::quaternionMsgToTF(msg->poses[i].pose.orientation, quat);
	    		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
			waypoint.yaw = yaw;
			waypoints.push_back(waypoint);
		}
		path->Add_new_point(waypoints);
		lastIndex = path->Size() - 1;
		lastPoint = path->Get_waypoint(lastIndex);
	}
 
	double slow_judge(double distance) {
		if (distance>=slow_LEVE2_DISTANCE&&distance <= slow_LEVE1_DISTANCE) {
			return slow_LEVE1_V;
		}
		else if (distance>=goal_tolerance_DISTANCE&&distance < slow_LEVE2_DISTANCE) {
			return slow_LEVE2_V;
		}
		else if (distance < goal_tolerance_DISTANCE) {
			action = "the car has reached the goal!";
			return 0.2;
		}
		else
		{
			return V_DESIRED;
		}
	}
 
	//The controller process

	void Controller_track(){
		//Search for path points
		//Construct the desired control amount
		//Use controller
		//topic release
		//Compare the state quantity with the expected value

		/*
			第一步，根据当前汽车的状态，找到在轨迹当中的最近点，
			根据最近点确定行车要走的目标状态，进而使用控制算法进行计算
			直接调用Path类的算法来解决
		*/
		ros::Time curTimer = ros::Time::now();
		
		//找到距离目标位置最近的点
		int path_index = this->path->Find_target_index(this->car);
		if(path_index == -1)
		{
			this->timer = curTimer;
			return ;
		}
		//如果已经在终点了，那么直接返回
		if(path_index == lastIndex) return ;

		double interval = curTimer.toSec()-this->timer.toSec();
		this->timer = curTimer;
		//获得目标点位的坐标状态
		waypoint target_node = this->path->Get_waypoint(path_index);
		U controller_param;

//  控制器相关,PID
#if(USE_PID_OR_LQR == 0)
{
		//设置控制器参数
		this->controller->initial(PID,interval,1.0,0.1,0.1,1.0,0.2,0.1);

		// if(path_index==lastIndex)
		// 	this->controller->initial(PID,interval,0,0.0,0.0,1.7,0.2,0.1);
		this->controller->param_struct(target_node.x,target_node.y,target_node.yaw);

		//根据控制器的算法，计算其控制变量，随后把控制变量丢给环境输出
		controller_param = this->controller->cal_vel(this->car);
		controller_param = this->v_and_kesi_limit(controller_param);
}
#endif

//  控制器相关,LQR
#if(USE_PID_OR_LQR == 1)
{
		//设置控制器参数
		this->controller->initial(LQR,Q_set,R_set,L,interval);

		// if(path_index==lastIndex)
		// 	this->controller->initial(PID,interval,0,0.0,0.0,1.7,0.2,0.1);
		double dis = sqrt(pow(target_node.x-car.x,2)+pow(target_node.y-car.y,2));
		this->controller->param_struct(target_node.x,target_node.y,target_node.yaw,
							slow_judge(dis),atan2(L*cal_K(this->path->Get_waypoints(),path_index),1) );

		//根据控制器的算法，计算其控制变量，随后把控制变量丢给环境输出
		controller_param = this->controller->cal_vel(this->car);
		controller_param = this->v_and_kesi_limit(controller_param);
}
#endif
		//cout<<controller_param.kesi<<' '<<controller_param.v<<endl;
		//更新数据并发布
		this->PUB();
		if(path_index!=lastIndex)
			this->car = update_state(controller_param,this->car,interval);
		//测试误差大小
		controller->test(target_node.x,target_node.y,this->car);
	}
	

	void node_control() {
		ros::Rate loop_rate(freq);
		Marker_set();

		tf::TransformBroadcaster br;
		tf::Transform transform;
		tf::Quaternion q;

		while (ros::ok()) {
			transform.setOrigin(tf::Vector3(car.x, car.y, 0));
			q.setRPY(0, 0, car.yaw);
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "car"));

			ros::spinOnce();
			Controller_track();
			loop_rate.sleep();
		}
	}

	void PUB(){
		visual_state_pose.x = car.x; visual_state_pose.y = car.y;
		actual_pose.x = car.x; actual_pose.y = car.y; actual_pose.theta = car.yaw;
		vel_msg.linear.x = control.v; vel_msg.angular.z = control.v*tan(control.kesi)/L;
		visual_state_trajectory.points.push_back(visual_state_pose);
		visual_state_pub.publish(visual_state_trajectory);
		vel_pub.publish(vel_msg);
		actual_state_pub.publish(actual_pose);
	}

	void shutdown_controller(){
		if(action == "the car has reached the goal!"){
			temp+=1;
			if(temp ==50){
				ROS_WARN("shutdown the controller!");
				temp = 0;
				ros::shutdown();
			}
		}
	}

	void Marker_set(){
	
		visual_state_trajectory.header.frame_id = "map";
		visual_state_trajectory.header.stamp = ros::Time::now();
		visual_state_trajectory.action = visualization_msgs::Marker::ADD;
		visual_state_trajectory.ns = "Control";
	
		visual_state_trajectory.id = 0;
		visual_state_trajectory.type = visualization_msgs::Marker::POINTS;
		visual_state_trajectory.scale.x = 0.02;
		visual_state_trajectory.scale.y = 0.02;
		visual_state_trajectory.color.r = 1.0;
		visual_state_trajectory.color.a = 1.0;
	}

	U v_and_kesi_limit(U control_value){
		if(control_value.v>=v_max)
		{
			control_value.v = v_max;
			// ROS_WARN("The calculated value may be inaccurate ");
		}
		else if(control_value.v<=-v_max){
			control_value.v = -v_max;
			// ROS_WARN("The calculated value may be inaccurate ");
		}
			

		if(control_value.kesi>=pi/2)
		{
			control_value.kesi = pi/2;
			// ROS_WARN("The calculated value may be inaccurate ");
		}
		else if(control_value.kesi<=-pi/2){
			control_value.kesi = -pi/2;
			// ROS_WARN("The calculated value may be inaccurate ");
		}
		return control_value;
	}
};
 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Control_node");
	ros::NodeHandle n;
	ros::NodeHandle n_prv("~");

	n_prv.param<double>("freq",freq,20);
	n_prv.param<double>("L",L,0.2);
	n_prv.param<double>("V_DESIRED",V_DESIRED,0.5);
	n_prv.param<double>("v_max",v_max,1.0);
	n_prv.param<double>("initial_x",initial_x,0.0);
	n_prv.param<double>("initial_y",initial_y,2.0);
	n_prv.param<double>("initial_yaw",initial_yaw,0.0);
	n_prv.param<double>("initial_v",initial_v,0.0);
	n_prv.param<double>("initial_kesi",initial_kesi,0.1);
	n_prv.param<double>("slow_LEVE1_DISTANCE",slow_LEVE1_DISTANCE,5.0);
	n_prv.param<double>("slow_LEVE2_DISTANCE",slow_LEVE2_DISTANCE,2.0);
	n_prv.param<double>("goal_tolerance_DISTANCE",goal_tolerance_DISTANCE,0.1);
	n_prv.param<double>("slow_LEVE1_V",slow_LEVE1_V,0.35);
	n_prv.param<double>("slow_LEVE2_V",slow_LEVE2_V,0.15);
	n_prv.param<bool>("limit_v_and_kesi",limit_v_and_kesi,false);

	// depends on the controller design
	n_prv.param("Q_set",Q_set,Q_set);
	n_prv.param("R_set",R_set,R_set);

	Control_node* node = new Control_node(n);
	node->node_control();
	return (0);
}

 
