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
#include <queue>
 
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

  void PopulateServersideBuffers();

  // Main function called continously from main
  void Run();

  std::tuple<float, float, float> GetPathScoringParams(float curvature_of_turning, Eigen::Vector2f& closest_point);

  Eigen::Vector2f TransformAndEstimatePointCloud(float x, float y, float theta, Eigen::Vector2f pt);

 private:

  // a vector of vector of points - this will be used as a buffer to store the scan values from multiple laser callbacks
  std::vector<std::vector<Eigen::Vector2f>> point_cloud_stack_;
  // a vector of ros::Time time_stamps - this will be used to store the ros time stamps corresponding to the muliple laser callbacks
  std::vector<ros::Time> point_cloud_time_stamp_stack_;

  // both point_cloud_stack_ and point_cloud_time_stamp_stack_ get emptied after every ros::spinonce
  // we need to filter out the required point clouds from the point_cloud_stack_ and assign them to a serverside point cloud buffer.
  // the first value of the serverside point cloud buffer will be used to generate path params for one call of the Run function. 
  // These path params will be transfered to the robot, thus for every 50 ms one value will be sent.

  std::vector<std::vector<Eigen::Vector2f>> serverside_point_cloud_buffer_;
  std::vector<ros::Time> serverside_point_cloud_time_stamp_buffer_;

  // required point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;
  //time stamp corresponding to the required point cloud
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

  // synchronization variables and constants 
  float scan_frequency = 40.0;
  float server_frequency = 20.0;
  int choose_after = static_cast<int>(scan_frequency/server_frequency);
  int choice_index = 0;

};

}  // namespace Serverside

#endif  // SERVERSIDE_H