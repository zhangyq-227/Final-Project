
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include "ros/ros.h"
#include <Eigen/Eigen>
// #include "reference_line.h"

namespace carla_pnc
{

  // 车辆位置信息
  struct car_state
  {
    double x;
    double y;
    double z;
    double yaw; 
    double vx;  
    double vy;  
    double v;   
    double cur; 
  };


  class Controller
  {
    
  public:
    /***********************************Vehicle Parameters**************************************/
    double L;         
    double cf;         
    double cr;         
    double mass_fl;    
    double mass_fr;   
    double mass_rl;    
    double mass_rr;    
    double mass_front; 
    double mass_rear;  
    double mass;       
    double lf;         
    double lr;         
    double Iz;         
    double max_degree; //aximum Front Wheel Steering Angle (degrees)"
  };
  typedef struct PIDController{
    double KpThrottle,KiThrottle,KdThrottle;
    double KpSteer,KiSteer,KdSteer;
    double KpBrake,KiBrake,KdBrake;

    double ErroThrottle, ErroSteer,ErroBrake;
    double preErroThrottle,preErroSteer,preErroBrake;

    double target_yaw;    //the yaw computed in the planning node is not the desired value! Fix this by myself
  } PIDController;
  car_state FindNearestPoint(car_state cur_state,std::vector<car_state> &local_waypoints);
  void InitialisePID(PIDController& PidControll,double KpT,double KiT,double KdT,
                    double KpS,double KiS,double KdS,
                    double KpB,double KiB,double KdB);
  double GetThrottleControl(PIDController& PidControll,car_state cur_state,car_state target_state,double duration);
  double GetSteerContorl(PIDController& PidControll,car_state cur_state,car_state target_state,double duration); 
  double GetBrakeControl(PIDController& PidControll,car_state cur_state,car_state target_state,double duration);
  std::vector<double> GetControlCmd(PIDController& PidControll,car_state cur_state,car_state target_state,double duration);
}

#endif // CONTROLLER_H
