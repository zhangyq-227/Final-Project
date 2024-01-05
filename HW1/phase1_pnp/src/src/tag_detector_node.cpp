#include <iostream>
#include<algorithm>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>
#include <board.h>
//EIgen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

using namespace cv;
using namespace Eigen;

//global varialbles for aruco detector
float MarkerSize = 0.20 / 1.5 * 1.524;
old_aruco::BoardConfiguration BoardConfig;
ros::Publisher pub_odom_yourwork;
ros::Publisher pub_odom_ref;
cv::Mat cameraMatrix, distCoeffs;
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

Eigen::Matrix3d pre_R = Eigen::Matrix3d::Zero();
Eigen::Vector3d pre_T = Eigen::Vector3d::Zero();

// test function, can be used to verify your estimation
void calculateReprojectionError(const std::vector<cv::Point3f> &pts_3, const std::vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{
    //puts("calculateReprojectionError begins");
    std::vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, cameraMatrix, distCoeffs);
    double avg_erro = 0.0f;
    double variance = 0.0f;
    vector<double> set_erro;
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        // printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
        //        pts_3[i].x, pts_3[i].y, pts_3[i].z,
        //        un_pts_2[i].x, un_pts_2[i].y,
        //        p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
        double dis = sqrt(pow(p.at<double>(0) / p.at<double>(2)-un_pts_2[i].x,2)+
                        pow(p.at<double>(1) / p.at<double>(2)-un_pts_2[i].y,2));
        avg_erro+=dis;
        set_erro.push_back(dis);
    }
    avg_erro/=pts_3.size();
    //compute variance
    for(int i = 0;i < set_erro.size();i++)
    {
        variance+=pow(set_erro[i]-avg_erro,2);
    }
    cout<<"variance: "<<variance/set_erro.size()<<' '<<"average erro: "<<avg_erro<<endl;
    //puts("calculateReprojectionError ends");
}

// the main function you need to work with
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const std::vector<cv::Point3f> &pts_3, const std::vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    //version 1, as reference
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, cameraMatrix, distCoeffs, rvec, t);
    cv::Rodrigues(rvec, r);
    Matrix3d R_ref;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            R_ref(i,j) = r.at<double>(i, j);
        }
    // cv::Mat target_R;
    // cv::eigen2cv(R_ref,target_R);
    // calculateReprojectionError(pts_3,pts_2,target_R,t);
    Quaterniond Q_ref;
    Q_ref = R_ref;
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "camera";
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    pub_odom_ref.publish(odom_ref);

    // version 2, your work
    Matrix3d R;
    Vector3d T;
    R.setIdentity();
    T.setZero();
    //ROS_INFO("write your code here!");

    //随机数种子，随机生成一个
    srand(time(NULL));
    //本质是在求解线性方程组
    vector<Point2f> new_point;
    //相机畸变矫正
    undistortPoints(pts_2,new_point,cameraMatrix,distCoeffs,cv::noArray(),cameraMatrix);
    int random_choice = (pts_3.size()*0.5>5)?pts_3.size()*0.5:5;  //随机选取几个点.这是一个超参数
    int iteration_num = 100; //设置ransac的迭代轮数
    double ransac_threshold = 0.02;  //设置ransac的阈值
    int cnt = 0;
    int max_correct_point = INT_MIN;
    double min_erro = INFINITY;
    Matrix3d tmpR;
    Vector3d tmpT;

    //利用RANSAC作最优解的拟合
    //仍然需要一些滤波算法来保证稳定性
    //EKF待添加
    while(((cnt++) < iteration_num) || (max_correct_point < pts_3.size()/2)){
        //step1，估计一个新的R，T矩阵
        MatrixXd EquMatrix(2*random_choice,9);
        //初始化全部为0
        EquMatrix.setZero();
        //初始化该矩阵，根据推导的表达式
        for(int i = 0;i < random_choice;i++)
        {
            int choice_index = rand()%pts_3.size();
            EquMatrix(2*i,0) = -1*pts_3[choice_index].x;
            EquMatrix(2*i,1) = -1*pts_3[choice_index].y;
            EquMatrix(2*i,2) = -1;
            EquMatrix(2*i,6) = new_point[choice_index].x*pts_3[choice_index].x;
            EquMatrix(2*i,7) = new_point[choice_index].x*pts_3[choice_index].y;
            EquMatrix(2*i,8) = new_point[choice_index].x;
            //设置下一行的参数
            EquMatrix(2*i+1,3) = -1*pts_3[choice_index].x;
            EquMatrix(2*i+1,4) = -1*pts_3[choice_index].y;
            EquMatrix(2*i+1,5) = -1;
            EquMatrix(2*i+1,6) = new_point[choice_index].y*pts_3[choice_index].x;
            EquMatrix(2*i+1,7) = new_point[choice_index].y*pts_3[choice_index].y;
            EquMatrix(2*i+1,8) = new_point[choice_index].y;
        }

        //求解方程组
        //利用SVD求解最小二乘
        JacobiSVD<MatrixXd> svd(EquMatrix, ComputeThinV);
        MatrixXd answer1 = svd.matrixV().col(svd.matrixV().cols()-1);
        Matrix3d answer;
        Matrix3d answer_3d;
        
        //将最终结果和k的逆矩阵相乘得到课件中的K^{-1}H
        for(int i = 0;i < 3;i++)
            for(int j = 0;j < 3;j++)
                answer_3d(i,j) = answer1(i*3+j,0);

        Matrix3d INVK;
        cv::cv2eigen(cameraMatrix,INVK);
        answer = INVK.inverse()*answer_3d;//K^{-1}H

        //将其和目标R之间作最小二乘
        Matrix3d  targetR;
        targetR.col(0) = answer.col(0);targetR.col(1) = answer.col(1);
        targetR.col(2) = answer.col(0).cross(answer.col(1));
        //targetR.col(1) = targetR.col(2).cross(targetR.col(0));
        JacobiSVD<MatrixXd> svd1(targetR, ComputeFullU | ComputeFullV);
        tmpR = svd1.matrixU()*svd1.matrixV().transpose();
        double sum_value = svd1.singularValues().sum();
        tmpT = answer.col(2)/answer.col(0).norm();
        //判断正负符号choice_index
        //这一项需要保证点在相机的前方
        int pos = 0;
        int neg = 0;
        for ( int i = 0; i < pts_3.size(); i ++ ) {
            const double& x = pts_3[i].x;
            const double& y = pts_3[i].y;
            const double& z = pts_3[i].z;

            double lambda = sum_value*(x*tmpR(2,0)+y*tmpR(2,1)+z*tmpR(2,2)+tmpT(2));
            if(lambda >= 0)
                pos++;
            else 
                neg++;
        }

        if ( pos < neg ) {
            tmpR = -1*tmpR;
            tmpT = -1*tmpT;
        }

        cv::Mat R1,T1;
        cv::eigen2cv(tmpR,R1);
        cv::eigen2cv(tmpT,T1);

        std::vector<cv::Point2f> un_pts_2;
        cv::undistortPoints(pts_2, un_pts_2, cameraMatrix, distCoeffs);

        int correct_num = 0;
        double avg_erro = 0.0;
        for (unsigned int i = 0; i < pts_3.size(); i++)
        {
            cv::Mat p_mat(3, 1, CV_64FC1);
            p_mat.at<double>(0, 0) = pts_3[i].x;
            p_mat.at<double>(1, 0) = pts_3[i].y;
            p_mat.at<double>(2, 0) = pts_3[i].z;
            cv::Mat p = (R1 * p_mat + T1);  //得到估计的点
            //计算估计点和去除畸变之后的点的距离是多少
            double dis = sqrt(pow(p.at<double>(0) / p.at<double>(2)-un_pts_2[i].x,2)+
                            pow(p.at<double>(1) / p.at<double>(2)-un_pts_2[i].y,2));
            avg_erro+=dis;
            if(dis <= ransac_threshold)
                correct_num++;
        }
        avg_erro/=pts_3.size();
        if(correct_num > max_correct_point){
            max_correct_point = correct_num;
            R=tmpR;
            T=tmpT;
        }
        else if(avg_erro < min_erro && correct_num==max_correct_point)
        {
            min_erro = avg_erro;
            R=tmpR;
            T=tmpT;
        }
        //cout<<pts_3.size()<<' '<<max_correct_point<<endl;
    }
    
    cv::Mat R1,T1;
    cv::eigen2cv(R,R1);
    cv::eigen2cv(T,T1);
    //计算反映射后的误差
    calculateReprojectionError(pts_3,pts_2,R1,T1);

    //...
    Quaterniond Q_yourwork;
    Q_yourwork = R;
    nav_msgs::Odometry odom_yourwork;
    odom_yourwork.header.stamp = frame_time;
    odom_yourwork.header.frame_id = "camera";
    odom_yourwork.pose.pose.position.x = T(0);
    odom_yourwork.pose.pose.position.y = T(1);
    odom_yourwork.pose.pose.position.z = T(2);
    odom_yourwork.pose.pose.orientation.w = Q_yourwork.w();
    odom_yourwork.pose.pose.orientation.x = Q_yourwork.x();
    odom_yourwork.pose.pose.orientation.y = Q_yourwork.y();
    odom_yourwork.pose.pose.orientation.z = Q_yourwork.z();
    pub_odom_yourwork.publish(odom_yourwork);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{

    double t = clock();

    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(bridge_ptr->image, dictionary, corners, ids);
    cv::Mat imageShow;
    bridge_ptr->image.copyTo(imageShow);

    // If at least one marker detected
    std::vector<cv::Point3f> pts_3;
    std::vector<cv::Point2f> pts_2;
    for(unsigned int i = 0; i < ids.size(); i++)
    {
        // push to PnP
        int id = ids[i];
        for (unsigned int j = 0; j < 4; j++)
        {
            pts_2.push_back( corners[i][j] );
            pts_3.push_back( BoardConfig.getMarkerInfo( id ).at( j ) * MarkerSize / 1000 );
        }
        // draw
        cv::aruco::drawDetectedMarkers(imageShow, corners, ids);
    }
    
    if (ids.size() > 5)
        process(pts_3, pts_2, img_msg->header.stamp);

    cv::imshow("image", imageShow);
    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);
    pub_odom_yourwork = n.advertise<nav_msgs::Odometry>("odom_yourwork",10);
    pub_odom_ref = n.advertise<nav_msgs::Odometry>("odom_ref",10);
    //init aruco detector
    std::string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    BoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> cameraMatrix;
    param_reader["distortion_coefficients"] >> distCoeffs;

    //init window for visualization
    cv::namedWindow("image", 1);

    ros::spin();
}