#include <iostream>
#include "Controller.h"
 
using namespace std;

// your code for update

//PID的参数控制
void Controller::initial(Controller_choice c,double time_internal,
				double kp1,double ki1,double kd1,
				double kp2,double ki2,double kd2) 
{
    //初始化选择PID参数
    if(c == PID)
    {
        this->cc = c;
        this->T = time_internal;
        this->kp1 = kp1;this->ki1=ki1;this->kd1 = kd1;
        this->kp2 = kp2;this->ki1=ki2;this->kd1 = kd2;
    }
}

//对应于LQR的控制器
void Controller:: initial(Controller_choice c,std::vector<double> Q,
                        std::vector<double> R,double L,double T)
{
    if(c==LQR)
    {
        this->T = T;
        this->L = L;
        //初始化Q，R向量
        //保证Q，R一定为正定矩阵
        this->cc = c;
        this->Q = Eigen::Matrix3d::Zero();
        this->R = Eigen::Matrix2d::Zero();
        for(int i = 0;i < 3;i++)    this->Q(i,i) = Q[i];
        for(int i = 0;i < 2;i++)    this->R(i,i) = R[i];
    }
}

 
//构建参数
void Controller::param_struct(double target_x,double target_y,double target_yaw) 
{
    if(this->cc == PID)
    {
        this->x_d = target_x;
        this->y_d = target_y;
        this->yaw_d = target_yaw;
        this->erro_angle_prev = this->erro_angle_sum = 0.0f;
        this->erro_dis_prev = this->erro_dis_sum = 0.0f;
        P = Matrix3x3::Ones();
    }
}

void Controller::param_struct(double target_x,double target_y,double target_yaw,
                  double target_v,double target_kesi)
{
    if(this->cc == LQR)
    {
        P = Q;
        if(target_v < 0 )
            target_kesi = -target_kesi;
        this->x_d = target_x;
        this->y_d = target_y;
        this->yaw_d = target_yaw;
        this->kesi_d = target_kesi;
        this->v_d = target_v;

        //根据目标值构造A，B矩阵
        A_d = Matrix3x3::Zero();
        B_d = Matrix3x2::Zero();

        A_d(0,2) = -1*v_d*sin(yaw_d);
        A_d(1,2) = v_d*cos(yaw_d);

        B_d(0,0) = cos(yaw_d);
        B_d(1,0) = sin(yaw_d);
        B_d(2,0) = tan(kesi_d)/L;
        B_d(2,1) = v_d/(L*cos(kesi_d)*cos(kesi_d));

        A_d*=T;
        B_d*=T;

        for(int i = 0;i < 3;i++) A_d(i,i) = 1.0f;
    }
}
 
U Controller::cal_vel(vehicleState car_state) {
    U control_val;
    this->x_car=car_state.x;
    this->y_car=car_state.y;
    this->yaw_car=car_state.yaw;
    
    if(this->cc == PID){
        //计算控制速度
        double erro_distance = sqrt(pow(car_state.x-this->x_d,2)+pow(car_state.y-y_d,2));
        double vKP = this->kp1*erro_distance;
        double vki = this->erro_dis_sum+ki1*T*(erro_distance+this->erro_dis_prev)/2;
        this->erro_dis_sum = vki;
        double vkd = this->kd1*((erro_distance-erro_dis_prev)/T);
        this->erro_dis_prev = erro_distance;

        control_val.v = vKP+vki+vkd;

        //计算控制角度变化
        double erro_angle = this->yaw_d - car_state.yaw;
        double aKP = this->kp2*erro_angle;
        double aki = this->erro_angle_sum+ki2*T*(erro_angle+this->erro_angle_prev)/2;
        this->erro_angle_sum = aki;
        double akd = this->kd2*((erro_angle-erro_angle_prev)/T);
        this->erro_angle_prev = erro_angle;

        control_val.kesi = aKP+aki+akd;
    }

    else if(this->cc == LQR)
    {
        Matrix2x3 K = this->cal_Riccati();
        // cout<<K<<endl;
        Vector2x1 ut,ustar;
        Vector3x1 xt,xstar;
        ustar[0] = v_d;
        ustar[1] = kesi_d;
        xt[0] = car_state.x;
        xt[1] = car_state.y;
        xt[2] = car_state.yaw;
        xstar[0] = x_d;
        xstar[1] = y_d;
        xstar[2] = yaw_d;
        ut = ustar + K*(xt - xstar);
        control_val.kesi = ut[1];
        control_val.v = ut[0];
        // cout<<K*(xt-xstar)<<endl;
        // cout<<K<<endl;
        // cout<<"LQR"<<endl;
    }

    return control_val;
}

//采用迭代的方法求解Ricatti方程
Matrix2x3 Controller::cal_Riccati()
{
    Matrix2x3 K;
    // 计算Riccati方程的数值解
    for(int i = 0;i < 20;i++)
    {
       // K = -1*(R+B_d.transpose()*P*B_d).inverse()*B_d.transpose()*P*A_d;
        P = Q+A_d.transpose()*P*A_d-A_d.transpose()*P*B_d
            *(R+B_d.transpose()*P*B_d).inverse()*B_d.transpose()
            *P*A_d;
    }
    K=-1*(B_d.transpose()*P*B_d+R).inverse()*B_d.transpose()*P*A_d;
    return K;
}

//测试函数
void Controller::test(double target_x,double target_y,vehicleState cur_state)
{
    double e = sqrt(pow(target_x-cur_state.x,2)+pow(target_y-cur_state.y,2));
    this->N++;
    this->avg_err = avg_err+(e-avg_err)/N;
    cout << "avg_erro: "<< avg_err<<endl;
}