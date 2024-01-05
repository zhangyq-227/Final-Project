#ifndef POINT_TYPES_H
#define POINT_TYPES_H

#include <array>
#include <iostream>
#include <string>
#include <vector>

namespace carla_pnc
{
  // Global coordinate system path point
  class path_point
  {
  public:
    double x;
    double y;
    double z;
    double yaw; // Yaw angle
    double cur; // Curvature
    double s_;  // Arc length
  };

  // Vehicle position information (global coordinate system trajectory point)
  class car_state : public path_point
  {
  public:
    double vx; // Velocity in the x-direction
    double vy; // Velocity in the y-direction
    double v;  // Total velocity

    double ax;
    double ay;
    double a; // Acceleration

    double t; // Time (relative)
  };

  // Frenet information added on top of global coordinate points
  class FrenetPoint : public car_state
  {
  public:
    double s;
    double l;

    // Time derivatives, dot derivatives
    double s_d; 
    double l_d; 
    double s_d_d;
    double l_d_d;
    double s_d_d_d;
    double l_d_d_d;

    // Derivatives with respect to s, prime derivatives
    double l_ds;
    double l_d_ds;
    double l_d_d_ds;
    double ds; // Used to calculate curvature

    // dp path used to calculate cost
    double dp_cost; // Cost from this point to the starting point
    int dp_pre_row; // Row number of the point with the minimum cost before this point
  };

  class FrenetPath
  {
  public:
    double cost = 0.0f;
    std::vector<FrenetPoint> frenet_path;
    int size_ = 0; // Used to record the number of valid points

    double max_speed;
    double max_acc;
    double max_curvature;
  };

} // carla_pnc

#endif // POINT_TYPES_H
