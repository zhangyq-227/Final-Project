#include "collision_detection/collision_detection.h"
using namespace std;

namespace carla_pnc
{
    /**
     * @brief Construct a new carla_pnc::CollisionDetection::CollisionDetection object
     *
     * @param detected_objects
     * @param collision_distance
     * @param ref_path
     */
    CollisionDetection::CollisionDetection(const std::vector<Obstacle> &detected_objects,
                                           const double &collision_distance,
                                           const std::vector<path_point> &ref_path)
    {
        this->detected_objects = detected_objects;
        this->collision_distance = collision_distance;
        this->ref_path = ref_path;
        static_obstacle_list.clear();
        dynamic_obstacle_list.clear();
        obstacle_classification(this->detected_objects);
    }

    /**
     * @brief Calculate obstacle collision BOX and classify (dynamic/static).
     *        Get the SL coordinates of static obstacles.
     *
     * @param detected_objects
     */
    void CollisionDetection::obstacle_classification(std::vector<Obstacle> &detected_objects)
    {
        for (Obstacle &obstacle : detected_objects)
        {
            // Calculate collision matrix
            cal_collision_box(obstacle);
            // ROS_INFO("Get collision_box successfully");
            if (obstacle.point.v > 0.2)
            {
                // for (auto &box_point : obstacle.collision_box)
                // {
                //     box_point = calc_frenet(box_point, ref_path);
                // }
                dynamic_obstacle_list.push_back(obstacle);
            }
            else
            { // Project static obstacles onto the SL graph
                obstacle.point = calc_frenet(obstacle.point, ref_path);
                // ROS_INFO("The obstacle,s:%.2f,l:%.2f",obstacle.point.s,obstacle.point.l);
                // ROS_INFO("The obstacle,x:%.2f,y:%.2f",obstacle.point.x,obstacle.point.y);
                for (auto &box_point : obstacle.collision_box)
                {
                    box_point = calc_frenet(box_point, ref_path);
                }
                static_obstacle_list.push_back(obstacle);
            }
        }
    }

    /**
     * @brief Get the collision BOX using 8 points representation.
     *
     * @param obstacle
     */
    void CollisionDetection::cal_collision_box(Obstacle &obstacle)
    {
        vector<FrenetPoint> collision_box(8);
        double x = obstacle.point.x;
        double y = obstacle.point.y;
        double yaw = obstacle.point.yaw;
        double x_rad = obstacle.x_rad;
        double y_rad = obstacle.y_rad;

        // Get the coordinates matrix of 8 points on the BOX edge
        Eigen::MatrixXd position_matrix(8, 2), translation_matrix(8, 2), rotation_matrix(2, 2);

        // Rotation matrix
        position_matrix << x, y,
            x, y,
            x, y,
            x, y,
            x, y,
            x, y,
            x, y,
            x, y;

        translation_matrix << -x_rad, -y_rad,
            -x_rad, 0,
            -x_rad, y_rad,
            0, y_rad,
            x_rad, y_rad,
            x_rad, 0,
            x_rad, -y_rad,
            0, -y_rad;

        rotation_matrix << cos(yaw), sin(yaw),
            -sin(yaw), cos(yaw);

        position_matrix = translation_matrix * rotation_matrix + position_matrix;
        
        for (int i = 0; i < position_matrix.rows(); i++)
        {
            collision_box[i].x = position_matrix(i, 0);
            collision_box[i].y = position_matrix(i, 1);
            collision_box[i].z = obstacle.point.z;
            collision_box[i].yaw = obstacle.point.yaw;
            collision_box[i].vx = obstacle.point.vx;
            collision_box[i].vy = obstacle.point.vy;
            collision_box[i].v = obstacle.point.v;
        }
        obstacle.collision_box = collision_box;
    }


    /**
     * @brief Collision detection and calculation of collision cost.
     *
     * @param path Frenet trajectory of the planned path
     * @param leader_point Target of the leading vehicle
     * @param car_following Car-following status
     * @return true if no collision, false if collision detected
     */
    bool CollisionDetection::check_collision(FrenetPath &path,
                                             const FrenetPoint &leader_point,
                                             const bool &car_following)
    {
        // Iterate over each obstacle
        double N=0;
        for (Obstacle obstacle : detected_objects)
        {
            // ROS_INFO("Obstacle point x:%.2f,y:%.2f",obstacle.point.x,obstacle.point.y);

            // Iterate over each point in the box
            for (auto box_point : obstacle.collision_box)
            {
                // ROS_INFO("box point x:%.2f, y:%.2f",box_point.x,box_point.y);
                // Iterate over each point on the path
                for (unsigned int i = 0; i < path.frenet_path.size(); i++)
                {
                    double dist = cal_distance(path.frenet_path[i].x, path.frenet_path[i].y,
                                               box_point.x, box_point.y);
                    // cout<<"dist to box: "<<dist<<endl;
                    if (dist <= collision_distance)
                    {
                        return false;
                    }

                    //能通过但是有一定的代价
                    // Calculate collision cost (excluding collision cost with the leading vehicle target)
                    if (dist < 3.5 &&
                        !(car_following && cal_distance(obstacle.point.x, obstacle.point.y, leader_point.x, leader_point.y) < 2.0))
                    {
                        //求出cost的平均值
                        N = N + 1.0f;
                        path.cost = path.cost+1.0f/N*(dist-path.cost);
                    }
                }
            }
        }
        return true;
    }
}