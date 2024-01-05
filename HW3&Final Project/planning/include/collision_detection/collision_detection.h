#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include "point_types.h"
#include "reference_line/reference_line.h"
#include "common.h"

namespace carla_pnc
{

    class Obstacle
    {
    public:
        FrenetPoint point;
        double x_rad;                         // X-axis radius
        double y_rad;                         // Y-axis radius
        std::vector<FrenetPoint> collision_box; // Collision box, represented by 8 points (4 vertices + 4 midpoints of edges)
    };

    class CollisionDetection
    {
    public:
        double collision_distance; // Collision distance
        std::vector<Obstacle> detected_objects;
        std::vector<Obstacle> static_obstacle_list;
        std::vector<Obstacle> dynamic_obstacle_list;
        std::vector<path_point> ref_path; // Reference line

        CollisionDetection() = default;

        CollisionDetection(const std::vector<Obstacle> &detected_objects,
                           const double &collision_distance,
                           const std::vector<path_point> &ref_path);

        void obstacle_classification(std::vector<Obstacle> &detected_objects);

        void cal_collision_box(Obstacle &object);

        bool check_collision(FrenetPath &path,
                             const FrenetPoint &leader_point,
                             const bool &car_following);
    };

} 

#endif
