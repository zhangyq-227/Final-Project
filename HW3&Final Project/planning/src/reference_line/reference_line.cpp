
#include "reference_line/reference_line.h"

using namespace std;

namespace carla_pnc
{


  /*******************************Class ReferenceLine ******************************************/

  /**
   * @brief Reference Line object
   *
   * @param lookahead_distance
   */
  ReferenceLine::ReferenceLine(double lookahead_distance,
                               std::unordered_map<std::string, double> &referline_params)
  {
    lookahead_dist = lookahead_distance;
    match_index = 0;


  }

  /**
   * @brief 
   *
   * @param cur_x
   * @param cur_y
   * @param path
   * @param lookahead_distance
   * @return int
   */
  int ReferenceLine::search_target_index(const double &cur_x, const double &cur_y,
                                         const std::vector<path_point> &path,
                                         const double &lookahead_distance)
  {
    double dist;
    for (int i = match_index; i < path.size(); i++)
    {
      dist = cal_distance(cur_x, cur_y, path[i].x, path[i].y);
      if (dist > lookahead_distance)
      {
        return i;
      }
    }
    return path.size() - 1;
  }

  /**
   * @brief 
   *
   * @param cur_pose 
   * @param global_path 
   * @param pre_match_index 
   * @return
   */
  std::vector<path_point> ReferenceLine::local_path_truncation(const car_state &cur_pose,
                                                               const std::vector<path_point> &global_path,
                                                               const int &pre_match_index)
  {
    this->match_index = search_match_index(cur_pose.x, cur_pose.y, global_path, pre_match_index);
    // ROS_INFO("Match point_index is %d:", match_index);

    int target_index = search_target_index(cur_pose.x, cur_pose.y, global_path, lookahead_dist);
    // ROS_INFO("Tatget point_index is %d:", target_index);

    vector<path_point> target_path(global_path.begin() + this->match_index,
                                   global_path.begin() + target_index + 1);
    // ROS_INFO("Size of target_path :%d", target_path.size());
    return target_path;
  }

  /**
   * @brief
   *
   * @param ref_frenet
   * @param local_path
   * @return 
   */
  std::vector<path_point> ReferenceLine::smoothing(Spline2D &ref_frenet, const std::vector<path_point> &local_path)
  {
    std::vector<path_point> ref_path;
    ref_path.clear();
    for (double i = 0; i < ref_frenet.s.back(); i += 0.1)
    {
      std::array<double, 2> point_ = ref_frenet.calc_postion(i);
      path_point ref_point;
      ref_point.x = point_[0];
      ref_point.y = point_[1];
      ref_point.yaw = ref_frenet.calc_yaw(i);
      ref_point.cur = ref_frenet.calc_curvature(i);
      //在frenet坐标系下的S坐标
      ref_point.s_ = i;
      ref_path.push_back(ref_point);
    }
    // ROS_INFO("The size of ref_path is:%zu", ref_path.size());
    return ref_path;
  }


  void ReferenceLine::cal_heading(vector<path_point> &waypoints)
  {
    double x_delta = 0.0;
    double y_delta = 0.0;
    double x_delta_2 = 0.0;
    double y_delta_2 = 0.0;
    for (int i = 0; i < waypoints.size(); i++)
    {
      if (i == 0)
      {
        x_delta = (waypoints[i + 1].x - waypoints[i].x);
        y_delta = (waypoints[i + 1].y - waypoints[i].y);
        x_delta_2 = (waypoints[i + 2].x - waypoints[i + 1].x) - (waypoints[i + 1].x - waypoints[i].x);
        y_delta_2 = (waypoints[i + 2].y - waypoints[i + 1].y) - (waypoints[i + 1].y - waypoints[i].y);
      }
      else if (i == waypoints.size() - 1)
      {
        x_delta = (waypoints[i].x - waypoints[i - 1].x);
        y_delta = (waypoints[i].y - waypoints[i - 1].y);
        x_delta_2 = (waypoints[i].x - waypoints[i - 1].x) - (waypoints[i - 1].x - waypoints[i - 2].x);
        y_delta_2 = (waypoints[i].y - waypoints[i - 1].y) - (waypoints[i - 1].y - waypoints[i - 2].y);
      }
      else
      {
        x_delta = 0.5 * (waypoints[i + 1].x - waypoints[i - 1].x);
        y_delta = 0.5 * (waypoints[i + 1].y - waypoints[i - 1].y);
        x_delta_2 = (waypoints[i + 1].x - waypoints[i].x) - (waypoints[i].x - waypoints[i - 1].x);
        y_delta_2 = (waypoints[i + 1].y - waypoints[i].y) - (waypoints[i].y - waypoints[i - 1].y);
      }
      waypoints[i].yaw = std::atan2(y_delta, x_delta);
   
      waypoints[i].cur = std::abs(y_delta_2 * x_delta - x_delta_2 * y_delta) / std::pow((x_delta * x_delta + y_delta * y_delta), 3 / 2);
    }
  }

} // namespace carla_pnc