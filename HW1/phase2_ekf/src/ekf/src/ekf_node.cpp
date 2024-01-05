#include <Eigen/Eigen>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#define _GRAVITY_ACC 9.8f
using namespace std;
using namespace Eigen;

ros::Publisher odom_pub;
ros::Publisher path_pub;
nav_msgs::Path path_ekf;
//微分的计算需要时间差值
ros::Time current_time, last_time;

//Imu带来的噪声方差，预测阶段有15个待估计参数
MatrixXd Q  = 25*MatrixXd::Identity( 12, 12 );
//视觉里程计带来的噪声的方差，测量值有6个参数
MatrixXd Rt = 2.0e-4*MatrixXd::Identity( 6, 6 );

//------------实际要更新的结果-----------------
//观测值，总共有十五个变量
Eigen::VectorXd observed_x = VectorXd::Zero(15);
//预测值的协方差矩阵
Eigen::MatrixXd sigma_t = Eigen::MatrixXd::Identity(15,15);

//----------每一次predict都是基于上一次predict--------------------
//---------predict在自身基础上更新20次，随后进入update-------------
Eigen::MatrixXd middle_sigma=sigma_t,middle_observed_x=observed_x;


//判断是否是第一次迭代进入
int cnt = 0;
Eigen::Matrix3d Rcam;

//完成扩展卡尔曼滤波的第一步，从Imu所得到的输入中预测下一步的结果
void
imu_callback( const sensor_msgs::Imu::ConstPtr& msg )
{
    cout<<"imu_time: "<<msg->header.stamp<<endl;
    //如果还没有拿到初始数据，直接推出,未初始化好的数据是有问题的
    if(!cnt)    return ;

    //拿到数据的时间戳
    current_time = msg->header.stamp;
    //减去时间戳获得秒数
    ros::Duration duration;
    duration = current_time - last_time;
    double delta_time = duration.toSec();
    if(delta_time < 0 || delta_time > 0.25)
        delta_time = 1.0/400.0;
    last_time = current_time;

    //----------------------更新预测值-------------------------------
    //-------------------------------------------------------------
    //创建一组临时变量
    Eigen::Vector3d measure_omega ;
    measure_omega << msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z;
    Eigen::Vector3d x4 ;
    x4 << observed_x(9),observed_x(10),observed_x(11);
    Eigen::Matrix3d R;
    double wmx=msg->angular_velocity.x,wmy=msg->angular_velocity.y,wmz=msg->angular_velocity.z;
    double amx = msg->linear_acceleration.x,amy=msg->linear_acceleration.y,amz=msg->linear_acceleration.z;

    /*
        将观测量拆分，分别表示，位置，欧拉角，线速度，角速度偏差，加速度偏差
    */
    double phi = observed_x(3),theta = observed_x(4),psi = observed_x(5), 
                p1=observed_x(0),p2=observed_x(1),p3=observed_x(2),
                p1d=observed_x(6),p2d=observed_x(7),p3d=observed_x(8),
                bg1=observed_x(9),bg2=observed_x(10),bg3=observed_x(11),
                ba1=observed_x(12),ba2=observed_x(13),ba3=observed_x(14);

    R(0,0) = cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta);
    R(0,1) = -cos(phi)*sin(psi);
    R(0,2) = cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi);
    R(1,0) = cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta);
    R(1,1) = cos(phi)*cos(psi);
    R(1,2) = sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi);
    R(2,0) = -cos(phi)*sin(theta);
    R(2,1) = sin(phi);
    R(2,2) = cos(phi)*cos(theta);
    
    Eigen::Vector3d x5,am,g;
    am << msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z;
    x5 << observed_x(12),observed_x(13),observed_x(14);
    g << 0.0f,0.0f,9.8f;

    Eigen::Matrix3d G = Eigen::Matrix3d::Zero();
    G(0,0) = cos(theta);
    G(2,0) = sin(theta);
    G(1,1) = 1;
    G(0,2) = -1*cos(phi)*sin(theta);
    G(1,2) = sin(phi);
    G(2,2) = cos(phi)*cos(theta);

    Eigen::VectorXd delta_x = Eigen::VectorXd::Zero(15);       //对观测值的微分更新项

    //填满这个矩阵
    for(int i = 0;i < 3;i++)    delta_x(i) = observed_x(i+6);
    Eigen::Vector3d second_row = G.inverse()*(measure_omega-x4);
    for(int i = 3;i < 6;i++)    delta_x(i) = second_row(i-3);
    Eigen::Vector3d third_row = g + R*(am-x5);
    for(int i = 6;i < 9;i++) delta_x(i) = third_row(i-6);

    //按照物理规律，预测一个新的x,更新的是中间变量，而不是最终的observed_x
    middle_observed_x = middle_observed_x + delta_time*delta_x;

    //-------------------------更新预测值的方差-------------------------------
    //---------------------------------------------------------------------
    //导入雅可比矩阵，可以从python的sympy中计算得到该矩阵
    double na1=0.0,ng1=0.0,na2=0.0,ng2=0.0,na3=0.0,ng3=0.0;
    Eigen::MatrixXd JacobiMatrix = Eigen::MatrixXd::Zero(15,15);    //雅可比矩阵，15x15维
    JacobiMatrix(0,6) =1;
    JacobiMatrix(1,7) =1;
    JacobiMatrix(2,8) =1;
    JacobiMatrix(3,4) =-(-bg1 - ng1 + wmx)*sin(theta) + (-bg3 - ng3 + wmz)*cos(theta);
    JacobiMatrix(3,9) =-cos(theta);
    JacobiMatrix(3,11) =-sin(theta);
    JacobiMatrix(4,3) =(tan(phi)*tan(phi) + 1)*(-bg1 - ng1 + wmx)*sin(theta) + (tan(phi)*tan(phi) + 1)*(bg3 + ng3 - wmz)*cos(theta);
    JacobiMatrix(4,4) =(-bg1 - ng1 + wmx)*cos(theta)*tan(phi) - (bg3 + ng3 - wmz)*sin(theta)*tan(phi);
    JacobiMatrix(4,9) =-sin(theta)*tan(phi);
    JacobiMatrix(4,10) =-1;
    JacobiMatrix(4,11) =cos(theta)*tan(phi);
    JacobiMatrix(5,3) =-(-bg1 - ng1 + wmx)*sin(phi)*sin(theta)/(cos(phi)*cos(phi)) + (-bg3 - ng3 + wmz)*sin(phi)*cos(theta)/(cos(phi)*cos(phi));
    JacobiMatrix(5,4) =-(-bg1 - ng1 + wmx)*cos(theta)/cos(phi) - (-bg3 - ng3 + wmz)*sin(theta)/cos(phi);
    JacobiMatrix(5,9) =sin(theta)/cos(phi);
    JacobiMatrix(5,11) =-cos(theta)/cos(phi);
    JacobiMatrix(6,3) =-(amx - ba1 - na1)*sin(psi)*sin(theta)*cos(phi) - (-amy + ba2 + na2)*sin(phi)*sin(psi) + (amz - ba3 - na3)*sin(psi)*cos(phi)*cos(theta);
    JacobiMatrix(6,4) =(-sin(phi)*sin(psi)*sin(theta) + cos(psi)*cos(theta))*(amz - ba3 - na3) + (-sin(phi)*sin(psi)*cos(theta) - sin(theta)*cos(psi))*(amx - ba1 - na1);
    JacobiMatrix(6,5) =(-sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(theta))*(amx - ba1 - na1) + (sin(phi)*cos(psi)*cos(theta) - sin(psi)*sin(theta))*(amz - ba3 - na3) + (-amy + ba2 + na2)*cos(phi)*cos(psi);
    JacobiMatrix(6,12) =sin(phi)*sin(psi)*sin(theta) - cos(psi)*cos(theta);
    JacobiMatrix(6,13) =sin(psi)*cos(phi);
    JacobiMatrix(6,14) =-sin(phi)*sin(psi)*cos(theta) - sin(theta)*cos(psi);
    JacobiMatrix(7,3) =(amx - ba1 - na1)*sin(theta)*cos(phi)*cos(psi) - (amy - ba2 - na2)*sin(phi)*cos(psi) - (amz - ba3 - na3)*cos(phi)*cos(psi)*cos(theta);
    JacobiMatrix(7,4) =(sin(phi)*sin(theta)*cos(psi) + sin(psi)*cos(theta))*(amz - ba3 - na3) + (sin(phi)*cos(psi)*cos(theta) - sin(psi)*sin(theta))*(amx - ba1 - na1);
    JacobiMatrix(7,5) =(-sin(phi)*sin(psi)*sin(theta) + cos(psi)*cos(theta))*(amx - ba1 - na1) + (sin(phi)*sin(psi)*cos(theta) + sin(theta)*cos(psi))*(amz - ba3 - na3) - (amy - ba2 - na2)*sin(psi)*cos(phi);
    JacobiMatrix(7,12) =-sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(theta);
    JacobiMatrix(7,13) =-cos(phi)*cos(psi);
    JacobiMatrix(7,14) =sin(phi)*cos(psi)*cos(theta) - sin(psi)*sin(theta);
    JacobiMatrix(8,3) =-(-amx + ba1 + na1)*sin(phi)*sin(theta) + (amy - ba2 - na2)*cos(phi) - (amz - ba3 - na3)*sin(phi)*cos(theta);
    JacobiMatrix(8,4) =(-amx + ba1 + na1)*cos(phi)*cos(theta) - (amz - ba3 - na3)*sin(theta)*cos(phi);
    JacobiMatrix(8,12) =sin(theta)*cos(phi);
    JacobiMatrix(8,13) =-sin(phi);
    JacobiMatrix(8,14) =-cos(phi)*cos(theta);
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(15,15)+delta_time*JacobiMatrix;
    //对噪声计算方差
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(15,12);
    V(3,0) = -cos(theta);
    V(3,2) = -sin(theta);
    V(4,0) = -sin(theta)*tan(phi);
    V(4,1) = -1;
    V(4,2) = cos(theta)*tan(phi);
    V(5,0) = sin(theta)/cos(phi);
    V(5,2) = -cos(theta)/cos(phi);
    V(6,3) = sin(phi)*sin(psi)*sin(theta) - cos(psi)*cos(theta);
    V(6,4) = sin(psi)*cos(phi);
    V(6,5) = -sin(phi)*sin(psi)*cos(theta) - sin(theta)*cos(psi);
    V(7,3) = -sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(theta);
    V(7,4) = -cos(phi)*cos(psi);
    V(7,5) = sin(phi)*cos(psi)*cos(theta) - sin(psi)*sin(theta);
    V(8,3) = sin(theta)*cos(phi);
    V(8,4) = -sin(phi);
    V(8,5) = -cos(phi)*cos(theta);
    V(9,6) = 1;
    V(10,7) = 1;
    V(11,8) = 1;
    V(12,9) = 1;
    V(13,10) = 1;
    V(14,11) = 1;
    V = delta_time*V;
    //更新方差，更新的是中间值，而不是最终结果
    middle_sigma = F*middle_sigma*F.transpose()+V*Q*V.transpose();
}

// Rotation from the camera frame to the IMU frame
//该函数接收接收另一个软件包即tag_detector的输入参数
//这一步目的在于根据观测值，做出新的预测
void
odom_callback( const nav_msgs::Odometry::ConstPtr& msg )
{   
    Eigen::Vector3d tran_from_cam_to_imu;
    tran_from_cam_to_imu << 0.05,0.05,0;

    // your code for update
    // camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							                 0, -1, 0,
    //                               0, 0, -1;
    
    //这里线性变换就可以表示，所以可以将其建模为一个完全线性的计算
    //此处需要首先处理一下观测数据
    /*
        tag_detector发布的数据是基于camear-world坐标系的，
        观测值observed_x是基于imu_world坐标系的，
        现在我们需要将其进行转化
    */

    //得到从world转向Imu的旋转矩阵与平移向量
    Eigen::MatrixXd Ct(6,15);
    Ct.setZero();
    for(int i = 0;i < 6;i++) Ct(i,i) = 1;
    Eigen::Quaterniond tmpq(msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);
    //得到测量值计算出的R，T矩阵，这一步是由tag_detector节点来完成的
    Eigen::Matrix3d rotMatrix = tmpq.toRotationMatrix();
    Eigen::Vector3d tranVec ;
    tranVec << msg->pose.pose.position.x,msg->pose.pose.position.y,
                msg->pose.pose.position.z;

    //-------------wc->wi-------------------------
    tranVec = Rcam*tranVec+tran_from_cam_to_imu;
    rotMatrix = Rcam*rotMatrix;
    //在对其求逆矩阵，得到T^w_i
    rotMatrix.transposeInPlace();
    tranVec = -1*rotMatrix*tranVec;
    Eigen::Vector3d eulerangle ;
    eulerangle = rotMatrix.eulerAngles(2,0,1);  //按照z-x-y顺序定义
    //拼接测量值
    Eigen::VectorXd zt(6);
    for(int i = 0;i < 3;i++) zt(i) = tranVec(i);
    zt(3) = eulerangle(1);zt(4) = eulerangle(2);zt(5) = eulerangle(0);
    
    //--------------------------更新新的----------------------------
    //如果是第一次的话直接相等就可以了
    if(cnt == 0){
        for(int i = 0;i < 6;i++)   observed_x(i) = zt(i);
        cnt++;
        last_time = msg->header.stamp;
        middle_observed_x = observed_x;
       cout<<"Initialize_: "<<last_time<<endl;
    }
    //不是第一次更新的话再计算其他的
    else{
        cout<<"odom: "<<msg->header.stamp<<endl;
        //计算卡尔曼增益
        Eigen::MatrixXd Kt = middle_sigma*Ct.transpose()*(Ct*middle_sigma*Ct.transpose()+Rt).inverse();
        //更新滤波后结果的方差
        sigma_t = middle_sigma - Kt*Ct*middle_sigma;
        observed_x = middle_observed_x+Kt*(zt-Ct*middle_observed_x);

        //for(int i = 0;i < 6;i++) middle_observed_x(i) = zt(i);
        middle_sigma = sigma_t;
        middle_observed_x = observed_x;
        //for(int i = 0;i < 6;i++) middle_observed_x(i) = zt(i);
    }

    //--------------------------------发布结果--------------------------
    Eigen::Vector3d ea1(observed_x(0),observed_x(1),observed_x(2));
    double phi = observed_x(3),theta = observed_x(4),psi = observed_x(5);
    Eigen::Matrix3d R;
    R(0,0) = cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta);
    R(0,1) = -cos(phi)*sin(psi);
    R(0,2) = cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi);
    R(1,0) = cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta);
    R(1,1) = cos(phi)*cos(psi);
    R(1,2) = sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi);
    R(2,0) = -cos(phi)*sin(theta);
    R(2,1) = sin(phi);
    R(2,2) = cos(phi)*cos(theta);
    //这里求得的是Imu坐标系相对于世界坐标系的关系，我们需要的是相机坐标系相对于世界坐标系之间的关系

    Quaterniond q;
    //将坐标系反变换回去
    Eigen::Matrix3d new_orientation = R*Rcam;
    new_orientation.transposeInPlace();
    q = new_orientation;
    Eigen::Vector3d trans = R*tran_from_cam_to_imu+ea1;
    trans = -1*new_orientation*trans;

    //定义发布数据
    nav_msgs::Odometry odom_yourwork;
    odom_yourwork.header.stamp = msg->header.stamp;
    odom_yourwork.header.frame_id = "camera";
    odom_yourwork.pose.pose.position.x = trans(0);
    odom_yourwork.pose.pose.position.y = trans(1);
    odom_yourwork.pose.pose.position.z = trans(2);
    odom_yourwork.pose.pose.orientation.w = q.w();
    odom_yourwork.pose.pose.orientation.x = q.x();
    odom_yourwork.pose.pose.orientation.y = q.y();
    odom_yourwork.pose.pose.orientation.z = q.z();
    odom_pub.publish(odom_yourwork);
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "ekf" );
    ros::NodeHandle n( "~" );

    current_time = last_time = ros::Time::now();
    ros::Subscriber s1 = n.subscribe( "imu", 1000, imu_callback );
    ros::Subscriber s2 = n.subscribe( "tag_odom", 1000, odom_callback );

    odom_pub = n.advertise< nav_msgs::Odometry >( "ekf_odom", 10 );
    path_pub = n.advertise< nav_msgs::Path >( "ekf_path", 10000 );

    Rcam = Quaterniond( 0, 1, 0, 0 ).toRotationMatrix( );
    cout << "R_cam" << endl << Rcam << endl;

    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    Q.topLeftCorner( 6, 6 )      = 0.1*Q.topLeftCorner( 6, 6 );
    Q.bottomRightCorner( 6, 6 )  = 0.1*Q.bottomRightCorner( 6, 6 );
    Rt.topLeftCorner( 3, 3 )     = 0.01*Rt.topLeftCorner( 3, 3 );
    Rt.bottomRightCorner( 3, 3 ) = 0.01*Rt.bottomRightCorner( 3, 3 );
    Rt.bottomRightCorner( 1, 1 ) = 0.01*Rt.bottomRightCorner( 1, 1 );

    path_ekf.header.frame_id = "world";

    ros::spin( );
}
