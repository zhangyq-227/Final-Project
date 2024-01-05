#ifndef EASY_PLANNER_H
#define EASY_PLANNER_H

#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include "collision_detection/collision_detection.h"
#include "common.h"
#include "reference_line/cubic_spline.hpp"
#include "reference_line/reference_line.h"
#include "point_types.h"


namespace carla_pnc{
//基于frenet坐标系下的规划器
    class Easy_Planner{
        private:
            std::vector<carla_pnc::path_point> InitialPath;     //初始的轨迹
            carla_pnc::FrenetPath pre_global_path;              //上一次计算的全局轨迹
            carla_pnc::car_state global_initial_state;          //全局的初始状态
            double collision_distance;                          //碰撞距离
            std::vector<carla_pnc::Obstacle> detected_obj;      //障碍物的位置信息
            double measure_range;                               //上下限的避障范围
            double measure_horizon;                             //视野上界
            double measure_range_lower;                         //视野下界
            std::vector<carla_pnc::FrenetPoint> timewalker;     //简易版时空走廊
            bool waiting = false;

        public:
            Easy_Planner();
            //执行规划器，得到一条新的轨迹
            carla_pnc::FrenetPath run();
            void set_measure_range(double range){
                measure_range = range;
            }
            void set_measure_horizon(double horizon){
                measure_horizon = horizon;
            }
            void set_measure_horizon_lower(double horizon){
                measure_range_lower = horizon;
            }
            void set_state_loop(carla_pnc::car_state global_initial_state,std::vector<carla_pnc::Obstacle> detected_obj,
                                double collision_distance,std::vector<carla_pnc::path_point> InitialPath);
            //找到最近的点,返回坐标
            int FindNearestPoint(car_state cur_state)
            {
                double MinDis=1000000.0f;
                int MinIndex = 0;
                for(int i = 0;i < this->pre_global_path.frenet_path.size();i++)
                {
                if(sqrt(pow(cur_state.x-pre_global_path.frenet_path[i].x,2)
                    +pow(cur_state.y-pre_global_path.frenet_path[i].y,2)) < MinDis){
                        MinDis = sqrt(pow(cur_state.x-pre_global_path.frenet_path[i].x,2)\
                                        +pow(cur_state.y-pre_global_path.frenet_path[i].y,2));
                        MinIndex = i;
                    }
                }
                return MinIndex; 
            }
            bool collision_check(FrenetPath &path)
            {
                for (auto box_point : this->timewalker)
                {
                    for (unsigned int i = 0; i < path.frenet_path.size(); i++)
                    {
                        double dist = cal_distance(path.frenet_path[i].x, path.frenet_path[i].y,
                                                box_point.x, box_point.y);
                        // cout<<"dist to box: "<<dist<<endl;
                        if (dist <= collision_distance)
                        {
                            return false;
                        }
                    }
                }
                return true;
            }
            double check_sign(double x){
                if(x > 0) return 1.0f;
                else if(x<0) return -1.0f;
                else return 0.0f;
            }
    };
}

#endif