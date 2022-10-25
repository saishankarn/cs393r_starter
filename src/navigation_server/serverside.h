//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
// 
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019 
*/
//========================================================================
 
#include <vector>
#include <fstream> 
#include "eigen3/Eigen/Dense"
#include <math.h>
 
#ifndef SERVERSIDE_H
#define SERVERSIDE_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace serverside {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Serverside {
 public:

   // Constructor
  explicit Serverside(const std::string& map_file, ros::NodeHandle* n);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         ros::Time time);

  // Main function called continously from main
  void Run();

  std::tuple<float, float, float> GetPathScoringParams(float curvature_of_turning, Eigen::Vector2f& closest_point);

  Eigen::Vector2f TransformAndEstimatePointCloud(float x, float y, float theta, Eigen::Vector2f pt);

 private:

  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;
  ros::Time point_cloud_time_stamp_;

  // kinematic variables 
  float max_acc = 4.0;
  float max_dec = 4.0;
  float max_vel = 1.0;
  float del_t = 0.05;
  const static int act_lat = 3;
  const static int sens_lat = 1;
  const static int net_lat =  5;
  const static int system_lat = sens_lat + act_lat;
  std::array<float, system_lat> vel_profile = {0};
  float vel_sum = 0;

  // navigation variables
  float length = 0.55;
  float width = 0.3;
  float wheel_base = 0.32;
  float track_width = 0.16;
  float safety_margin = 0.05; 

  Eigen::Vector2f center_of_curve;

};

}  // namespace Serverside

#endif  // SERVERSIDE_H