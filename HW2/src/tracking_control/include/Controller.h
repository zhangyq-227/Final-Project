#include <iostream>
#include <Eigen/Dense>
#include "Tool.h"
using namespace std;
 
typedef Eigen::Matrix<double, 3, 3> Matrix3x3;
typedef Eigen::Matrix<double, 3, 1> Matrix3x1;
typedef Eigen::Matrix<double, 2, 1> Matrix2x1;
typedef Eigen::Matrix<double, 2, 2> Matrix2x2;
typedef Eigen::Matrix<double, 3, 2> Matrix3x2;
typedef Eigen::Matrix<double, 2, 3> Matrix2x3;
typedef Eigen::Vector3d Vector3x1;
typedef Eigen::Vector2d Vector2x1;
enum Controller_choice{PID=0,LQR=1};
 
//state variables: X = [x_e  y_e  yaw_e]^T
//control input: U = [v_e  kesi_e]^T
 
class Controller
{
private:

	double L;//vehicle wheelbase
	double T;//sampling interval
	double x_car=0.0, y_car=0.0, yaw_car=0.0, x_d=0.0, y_d=0.0, yaw_d=0.0;//vehicle pose and target point pose
	double v_d=0.0, kesi_d=0.0;//desired speed and front wheel deviation angle

	int temp = 0;

	//depending on the controller design
	Matrix3x3 A_d;
	Matrix3x2 B_d;
	Matrix3x3 Q;
	Matrix2x2 R;
	Matrix3x1 X_e;
	Matrix2x1 U_e;
	Matrix3x3 P;

	//PID parameters
	double kp1,ki1,kd1,kp2,ki2,kd2;
	//用作速度控制中对I和D的误差计算
	double erro_dis_sum,erro_dis_prev ;
	//用作角度闭环中对I和D的误差计算
	double erro_angle_sum,erro_angle_prev;
	Controller_choice cc;

	//误差平均值大小，以及计算次数，用来采用增量式计算误差
	double avg_err = 0.0f;
	int N = 0;
	
public:
	// PID控制器初始化
	void initial(Controller_choice c,double time_internal,
				double kp1,double ki1,double kd1,	//这一组参数是用来将位置转化为速度的
				double kp2,double ki2,double kd2);	//这一组参数是用来将角度转化为kesi的
	// LQR控制器初始化
	void initial(Controller_choice c,std::vector<double> Q,std::vector<double> R,
				double L,double T);
	void param_struct(double target_x,double target_y,double target_yaw);
	void param_struct(double target_x,double target_y,double target_yaw,
					  double target_v,double target_kesi);
	Matrix2x3 cal_Riccati();
	U cal_vel(vehicleState car_state);//Caculate the control 
	void test(double target_x,double target_y,vehicleState cur_state);
};
 
 
 