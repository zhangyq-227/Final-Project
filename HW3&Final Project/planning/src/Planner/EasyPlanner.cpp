
/*
*****************************************************************************
*  @brief   Planning algorithm for mobile robot course project. FAET,FDU 2023 Autumn
*
*  @author  Yueqi Zhang
*  @date    2023-12-28
*  @version v0.1
*
*****************************************************************************
*/

#include "Planner/EasyPlanner.h"

namespace carla_pnc{

    //Construction Function
    Easy_Planner::Easy_Planner()
    {
        this->measure_range = 8.0f;                 //default range
        this->measure_range_lower = -3.0f;         //default range
        this->measure_horizon = 50.0f ;           //default horizon
    }

    void Easy_Planner::set_state_loop(carla_pnc::car_state global_initial_state,std::vector<carla_pnc::Obstacle> detected_obj,
                        double collision_distance,std::vector<carla_pnc::path_point> InitialPath)
    {
        this->collision_distance = collision_distance;
        this->global_initial_state = global_initial_state;
        this->InitialPath = InitialPath;
        this->detected_obj = detected_obj;
    }

    /*
        执行一次规划器的算法，获得新生成的局部轨迹
    */
    carla_pnc::FrenetPath Easy_Planner::run()
    {
        //候选曲线组
        std::vector<FrenetPath> candidate_Lines;

        //碰撞检测
        carla_pnc::CollisionDetection collisiondetection(detected_obj,collision_distance,InitialPath);

        //默认左转右转上下限
        double default_upper = measure_range;
        double default_lower = measure_range_lower;

        timewalker.clear();
        //生成一个时空走廊，作为预测量
        for (Obstacle obstacle : collisiondetection.detected_objects)
        {
            for (auto box_point : obstacle.collision_box)
            {
                for(double dt = 0.0;dt<=2.0f;dt+=0.1f)
                {
                    FrenetPoint p;
                    p.x = box_point.x+dt*box_point.vx;
                    p.y = box_point.y+dt*box_point.vy;
                    p.vx = box_point.vx;
                    p.vy = box_point.vy;
                    p.v = box_point.v;
                    this->timewalker.push_back(p);
                }
            }
        }

        /*------------------------------step0-------------------------------*/        
        //首先检查上一次曲线，查看当前的环境会不会和上一次曲线碰上,希望保持当前规划执行而不是立马换一条新的曲线
        FrenetPoint p;

        int nearest_point_index = FindNearestPoint(global_initial_state);
        
        if(nearest_point_index >= 0.5*pre_global_path.frenet_path.size() || (pre_global_path.frenet_path[0].v<0.2f || \
            pre_global_path.frenet_path[0].v>=20.0f) )
        {
            // std::cout<<nearest_point_index<<"   "<<pre_global_path.frenet_path.size();
            // std::cout<<"change another way"<<std::endl;
            goto REPLAN;
        }

        //当还没有经过80%路径点的时候,检测是否产生碰撞，如果会产生碰撞的话，将会还一条新的路径
        for(auto box_point: this->timewalker){
            for (unsigned int i = nearest_point_index; i < pre_global_path.frenet_path.size(); i++)
            {
                double dist = cal_distance(pre_global_path.frenet_path[i].x, pre_global_path.frenet_path[i].y,
                                            box_point.x, box_point.y);
                if (dist <= collision_distance)
                {
                    std::cout<<"can't follow last way!"<<std::endl;
                    goto REPLAN;    //碰撞后跳转到planning模块规划一条新的路径
                }
            }
        }

        //当不需要重新规划的时候,将上一次的path返回继续跟踪
        // std::cout<<"following last road"<<std::endl;
        return this->pre_global_path;

REPLAN:
        /*<--------------------------step1---------------------------------------> */
        //step1. 找到参考曲线上到当前点的最近点，找到沿轨迹向前50m的范围作为参考曲线
        int Line_end,Line_index;
        for(Line_index = 0;Line_index < InitialPath.size();Line_index++)
        {
            if(InitialPath[Line_index].s_ > this->measure_horizon)
                break;
        }
        Line_end = Line_index;

        /*<--------------------------step2------------------------------------------>*/
        //step2. 进行轨迹采样，采用frenet坐标系进行
        FrenetPoint frenet_initial_point = calc_frenet(this->global_initial_state,this->InitialPath);
        
        //判断当前前向碰撞点的车的速度和当前速度的关系，当逆向的时候尽量从右侧躲开
        for(int i = 0;i < Line_end;i++)
        {
            //获取下一个点的坐标
            FrenetPoint next_p ;
            next_p.s = InitialPath[i].s_;
            next_p.l = frenet_initial_point.l;
            next_p.x = InitialPath[i].x - next_p.l*sin(InitialPath[i].yaw);
            next_p.y = InitialPath[i].y + next_p.l*cos(InitialPath[i].yaw);

            //查看下一个点距离障碍物有无碰撞
            for (auto box_point : this->timewalker)
            {
                    double dist = cal_distance(next_p.x, next_p.y,
                                                box_point.x, box_point.y);
                    if (dist <= collision_distance)
                    {
                        //遇到碰撞点,查看速度方向进而确定向哪个方向躲避障碍物
                        double box_vx = box_point.vx;
                        double box_vy = box_point.vy;
                        double cur_vx = check_sign(global_initial_state.yaw);
                        double cur_vy = check_sign(cos(global_initial_state.yaw));

                        double Multiply = box_vx*cur_vx + box_vy*cur_vy;
                        double cosx = Multiply/(hypot(box_vx,box_vy)*hypot(cur_vx,cur_vy));
                        //前方车辆直接静止不动，检测到需要左侧超车
                        if(box_point.v < 0.2) {
                            default_upper = this->measure_range;
                            default_lower = 0.0f;
                            std::cout<<"choose 0"<<std::endl;
                        }
                        //横冲直撞的话，如果不行就直接减速
                        else if(fabs(cosx)<0.1){
                            default_upper = 0.0f;
                            default_lower = 0.0f;
                            std::cout<<"choose 1 "<<std::endl;
                        }
                        else if(cosx < 0){
                            default_upper = 0.0f;
                            default_lower = -this->measure_range_lower;
                            std::cout<<"choose 2 "<<std::endl;
                        }
                        else{
                            default_upper = this->measure_range;
                            default_lower = 0.0f;
                            std::cout<<"choose 3 "<<Multiply<<std::endl;
                        }
                        goto SAMPLE;
                    }
                }
        }

        std::cout<<"don't need to avoid !"<<std::endl;

SAMPLE:
        //根据最远端距离当前点的不同的偏离程度来采样获取一系列的点
        //每20cm划分一次，规划器规划正负5m内的距离偏差
        for(double dev_lateral = fabs(default_upper);dev_lateral >= -fabs(default_lower);dev_lateral-=0.1f){

            //按照不同的样条曲率的曲线程度再次进行分类
            for(double i = 0.0;i <= 0.3;i += 0.1){

                FrenetPath new_path;        //定义新的采样路径
                std::vector<double> x_set,y_set; //定义X轴和y轴的坐标
                double target_d = frenet_initial_point.l + dev_lateral;
                double Line_size = (double)Line_end;
                int Start_Point = (int)(i*Line_size);
                int Last_Point = (int)((1-i)*Line_size);
                double RisingLineSize = Last_Point - Start_Point;

                for (int j = 0;j <= Line_size;j+=10){
                    
                    FrenetPoint newPoint;

                    if(j <= Start_Point) {newPoint.s = InitialPath[j].s_; newPoint.l = frenet_initial_point.l;}
                    
                    else if(j >= Last_Point){newPoint.s = InitialPath[j].s_; newPoint.l = target_d;}
                    
                    else{
                        newPoint.s = InitialPath[j].s_;
                        newPoint.l = frenet_initial_point.l+(j-Start_Point)/RisingLineSize*dev_lateral;
                    }
                    
                    //转为全局坐标系
                    newPoint.x = InitialPath[j].x - newPoint.l*sin(InitialPath[j].yaw);
                    newPoint.y = InitialPath[j].y + newPoint.l*cos(InitialPath[j].yaw);
                    x_set.push_back(newPoint.x);
                    y_set.push_back(newPoint.y);

                }

                //采用三次样条插值恢复原来的采样间距，这一步是让轨迹更加平缓
                //TODO 更平缓的插值操作
                Spline2D ref_frenet(x_set,y_set);
                //按照每隔0.1的距离重新采样生成一条光滑的曲线
                for (double i = 0; i < ref_frenet.s.back(); i += 0.1)
                {
                    FrenetPoint NewFrenetPoint;
                    std::array<double, 2> point_ = ref_frenet.calc_postion(i);
                    path_point ref_point;
                    ref_point.x = point_[0];
                    ref_point.y = point_[1];
                    ref_point.yaw = ref_frenet.calc_yaw(i);

                    //计算frenet坐标系下的点
                    carla_pnc::car_state insert_state;
                    insert_state.x = ref_point.x;
                    insert_state.y = ref_point.y;
                    insert_state.yaw = ref_point.yaw;
                    insert_state.v = 11.0f;

                    //这几个变量不需要，因此直接给成0即可
                    insert_state.ax = insert_state.vy = insert_state.ay = insert_state.vx = 0.0f;
                    
                    NewFrenetPoint = calc_frenet(insert_state,this->InitialPath);
                    new_path.frenet_path.push_back(NewFrenetPoint);
                }

                candidate_Lines.push_back(new_path);
            }
        }

        /*<-----------------------------step3-------------------------------->*/
        //step3. 碰撞检测与轨迹判断,选择一条cost最小的轨迹
        bool all_false = true;
        int MinIndex = 0;
        double MinVal = 1000000.0f;

        for(int i = 0;i < candidate_Lines.size();i++)
        {
            //目前不工作在跟随模式下，因此第二个参数实际上是没有用到的
            FrenetPoint p;
            //若有障碍物遮挡，规划失败，重新换一条新的路线
            if(!collision_check(candidate_Lines[i])) 
            {
                // ROS_INFO("Can't avoid! Try another way! ");
                continue;
            }

            //若能够通过的话，在能够通过的所有曲线当中寻找一个代价最小的曲线，同时规划需要考虑跟踪ref的误差，尽可能贴近ref
            all_false = false;
            
            double deviate_origin_error = 0.0f;

            for(int j = 0;j < candidate_Lines[i].frenet_path.size();j++)
                deviate_origin_error += fabs(candidate_Lines[i].frenet_path[j].l);
            deviate_origin_error /= candidate_Lines[i].frenet_path.size();

            double cost = deviate_origin_error*0.4 + 0.6*candidate_Lines[i].cost;
            // std::cout<<"cost"<<i<<':'<<cost<<std::endl;

            if(cost < MinVal){MinIndex = i;MinVal=cost;}
        }

        FrenetPath ans;
        if(all_false){
            //当所有的轨迹都不能完成任务的时候，有两种方案，要么急刹车，要么加速过去
            //需要判断当前车辆与碰撞物体与当前位置姿态的夹角，如果夹角太大说明车辆已经形式到前方，加速冲过去
            double defined_speed=0.0f;

            if(!waiting){
                    for(int i = 0;i < Line_end;i++)
                    {
                        //获取下一个点的坐标
                        FrenetPoint next_p ;
                        next_p.s = InitialPath[i].s_;
                        next_p.l = frenet_initial_point.l;
                        next_p.x = InitialPath[i].x - next_p.l*sin(InitialPath[i].yaw);
                        next_p.y = InitialPath[i].y + next_p.l*cos(InitialPath[i].yaw);

                        //查看下一个点距离障碍物有无碰撞
                        for (auto box_point : this->timewalker)
                        {
                                double dist = cal_distance(next_p.x, next_p.y,
                                                            box_point.x, box_point.y);
                                if(dist < collision_distance && i < 0.1*Line_end)
                                    defined_speed=25.0f;
                        }
                    }
            }

            for(int i = 0;i < Line_end;i++){
                FrenetPoint p;
                p.s = InitialPath[i].s_;
                p.l = frenet_initial_point.l;
                p.x = InitialPath[i].x - p.l*sin(InitialPath[i].yaw);
                p.y = InitialPath[i].y + p.l*cos(InitialPath[i].yaw);
                p.v = defined_speed;
                ans.frenet_path.push_back(p);
            }
            if(defined_speed < 0.2f) waiting = true;
            ROS_INFO("Can't Avoid! Waiting...");
        }
        
        // 如果有路径能通过的话，那么选择一条cost function较小的路径作为目标
        else{
            ans = candidate_Lines[MinIndex];
            waiting=false;
        }

        pre_global_path = ans;
        // std::cout<<pre_global_path.frenet_path.size()<<"  "<<ans.frenet_path.size()<<std::endl;
        return  ans;     
    }
}