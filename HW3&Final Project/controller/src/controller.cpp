
/*
*****************************************************************************
*  @brief   Controller algorithm for mobile robot course project. FAET,FDU 2023 Autumn
*
*  @author  Yueqi Zhang
*  @date    2023-12-28
*  @version v0.1
*
*****************************************************************************
*/

#include "../include/controller.h"

using namespace std;

namespace carla_pnc
{
  int sign(double x){return (x>=0)?1:-1;}
  //找到最近的点
  car_state FindNearestPoint(car_state cur_state,std::vector<car_state> &local_waypoints)
  {
    double MinDis=1000000.0f;
    int MinIndex = 0;
    for(int i = 0;i < local_waypoints.size();i++)
    {
      if(sqrt(pow(cur_state.x-local_waypoints[i].x,2)
          +pow(cur_state.y-local_waypoints[i].y,2)) < MinDis){
            MinDis = sqrt(pow(cur_state.x-local_waypoints[i].x,2)+pow(cur_state.y-local_waypoints[i].y,2));
            MinIndex = i;
          }
    }

    //目标角度定义成一个加权之和
    //找到前向视野当中转角最大的
    double target_yaw = -1000000.0f;
    double MaxYaw = 0.0f;
    //需要提前做出反应，不然难以应对急转弯
    for(int i = MinIndex+40;i <MinIndex+50 && i<local_waypoints.size();i++)
    {
      double tar_yaw=atan2(local_waypoints[i].y-cur_state.y,local_waypoints[i].x-cur_state.x);
      double dev_yaw = 0.0f;
      if(cur_state.yaw * tar_yaw> 0) dev_yaw = fabs(cur_state.yaw-tar_yaw);
      else if(cur_state.yaw * tar_yaw < 0){
          double inverse_clock = fabs(cur_state.yaw)+fabs(tar_yaw);
          if(inverse_clock >= 1.57) dev_yaw = (2*3.1415926-inverse_clock);
          else dev_yaw = fabs(cur_state.yaw - tar_yaw);
      }
      if(dev_yaw > target_yaw) {target_yaw = dev_yaw; MaxYaw=tar_yaw;}
    }

    // cout<<"target_yaw: "<<MinYaw<<endl;

    local_waypoints[MinIndex].yaw = MaxYaw;
    //cout<<"correct :"<<correct_angle<<endl;
    // cout<<"MinDis: "<<MinDis<<endl;
    return local_waypoints[MinIndex];     
  }

  //初始化PID参数
  void InitialisePID(PIDController& PidControll,double KpT,double KiT,double KdT,
                    double KpS,double KiS,double KdS,
                    double KpB,double KiB,double KdB)
  {
        PidControll.KpThrottle=KpT;
        PidControll.KpSteer=KpS;
        PidControll.KpBrake=KpB;
        PidControll.KiThrottle=KiT;
        PidControll.KiSteer=KiS;
        PidControll.KiBrake=KiB;
        PidControll.KdThrottle=KdT;
        PidControll.KdSteer=KdS;
        PidControll.KdBrake=KdB;

        PidControll.ErroBrake = 0.0f;
        PidControll.ErroSteer = 0.0f;
        PidControll.ErroThrottle = 0.0f;

        PidControll.preErroBrake = 0.0f;
        PidControll.preErroSteer = 0.0f;
        PidControll.preErroThrottle = 0.0f;        
  }

  double GetThrottleControl(PIDController &PidControll,car_state cur_state,car_state target_state,double duration)
  {
      //用速度反馈信号控制汽车的油门
      double erro = target_state.v - cur_state.v;

      // cout<<"targetv: "<<target_state.v<<" "<<cur_state.v<<endl;

      double KP = PidControll.KpThrottle*erro;
      double KI = PidControll.ErroThrottle+PidControll.KiThrottle*duration*\
                (PidControll.preErroThrottle+erro)/2.0f;
      double KD = PidControll.KdThrottle*(erro-PidControll.preErroThrottle)/duration;
      PidControll.preErroThrottle = erro;
      PidControll.ErroThrottle = KI;

      if(KI > 1.0f) KI = 1.0f; else if(KI < 0.0f) KI = 0.0f;

      double res = KP + KI + KD;
      if(res > 1.0f) res = 1.0f; else if(res < 0.0f) res = 0.0f;

      return res;
  }

  double GetSteerContorl(PIDController& PidControll,car_state cur_state,car_state target_state,double duration)
  {
      //根据carla当中的坐标系定义，有四种情况需要考虑
      double erro =0.0f;

      if(cur_state.yaw * target_state.yaw> 0) {
        erro=cur_state.yaw-target_state.yaw;
      }
      else if(cur_state.yaw * target_state.yaw < 0){
          double inverse_clock = fabs(cur_state.yaw)+fabs(target_state.yaw);
          if(inverse_clock >= 1.57) erro = sign(target_state.yaw - cur_state.yaw)*(2*3.1415926-inverse_clock);
          else erro = cur_state.yaw - target_state.yaw;
      }

      // std::cout<<cur_state.yaw<<' '<<atan(cur_state.y/cur_state.x)<<' '<<target_state.yaw<<' '<<atan(target_state.y/target_state.x)<<endl;

      double KP=PidControll.KpSteer*erro;
      double KI=PidControll.ErroSteer+PidControll.KiSteer*duration*(PidControll.preErroSteer+erro)/2.0f;
      double KD=PidControll.KdSteer*(erro-PidControll.preErroSteer)/duration;
      PidControll.preErroSteer = erro;
      PidControll.ErroSteer = KI;

      if(KI > 1.0f) KI=1.0f; else if(KI < -1.0f) KI=-1.0f;

      double res=KP+KI+KD;
      if(res > 1.0f) res=1.0f; else if(res < -1.0f) res=-1.0f;

      return res;
  }

  double GetBrakeControl(PIDController &PidControll,car_state cur_state,car_state target_state,double duration)
  {
      double erro = cur_state.v - target_state.v;

      double KP = PidControll.KpBrake*erro;
      double KI = PidControll.ErroBrake+PidControll.KiBrake*duration*\
      (PidControll.preErroBrake+erro)/2.0f;
      double KD = PidControll.KdBrake*(erro-PidControll.preErroBrake)/duration;
      PidControll.preErroBrake = erro;
      PidControll.ErroBrake = KI;

      if(KI > 1.0f) KI = 1.0f; else if(KI < 0.0f) KI = 0.0f;

      double res = KP + KI + KD;
      if(res > 1.0f) res = 1.0f; else if(res < 0.0f) res = 0.0f;
      return res;
  }
 
  std::vector<double> GetControlCmd(PIDController &PidControll,car_state cur_state,car_state target_state,double duration)
  {
    std::vector<double> ans={0,0,0};

    //油门和刹车不能同时为正数
    ans[0] = GetThrottleControl(PidControll,cur_state,target_state,duration);
    ans[1] = GetSteerContorl(PidControll,cur_state,target_state,duration);
    ans[2] = GetBrakeControl(PidControll,cur_state,target_state,duration);

    //无论如何也无法躲开，紧急采取刹车
    if(target_state.v < 0.2) {
      ans[0] = 0.0f;ans[2]=1.0f;
      return ans;
    }

    //当转弯时，减速慢行,适当踩刹车
    if(fabs(ans[1])>0.2)  ans[0] = ans[0]*exp(-fabs(ans[1]));
    if(fabs(ans[1])>0.5)  {ans[2]=exp(-fabs(ans[1]));ans[0]=0.0f;}

    return ans;
  }
  // depend on your HW2
}