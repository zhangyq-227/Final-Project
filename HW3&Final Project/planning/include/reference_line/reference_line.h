
#ifndef REFERENCELINE_H
#define REFERENCELINE_H

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include "OsqpEigen/OsqpEigen.h"
#include "point_types.h"
#include "reference_line/cubic_spline.hpp"
#include "common.h"

namespace carla_pnc
{

  class ReferenceLine
  {
  public:
    double lookahead_dist;
    int match_index;

    ReferenceLine(double lookahead_distance) : lookahead_dist(lookahead_distance), match_index(0) {}

    ReferenceLine(double lookahead_distance,
                  std::unordered_map<std::string, double> &referline_params);

    int search_target_index(const double &cur_x, const double &cur_y,
                            const std::vector<path_point> &waypoints,
                            const double &lookahead_distance);

    std::vector<path_point> local_path_truncation(const car_state &cur_pose,
                                                  const std::vector<path_point> &global_path,
                                                  const int &pre_match_index);

    // cublic Spiline 
    std::vector<path_point> smoothing(Spline2D &ref_frenet,
                                      const std::vector<path_point> &local_path);

    void cal_heading(std::vector<path_point> &waypoints);

  private:


  };
} // namespace carla_pnc
#endif // REFERENCELINE_H
