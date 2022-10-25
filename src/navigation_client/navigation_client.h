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
\file    navigation_client.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================
 
#include <vector>
#include <fstream>
#include "eigen3/Eigen/Dense"
#include <math.h>

#ifndef NAVIGATION_CLIENT_H
#define NAVIGATION_CLIENT_H

namespace ros { 
  class NodeHandle;
}  // namespace ros

namespace navigation_client {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation_client {
 public:

   // Constructor
  explicit Navigation_client(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback to assign the path params corresponding to a delayed laser scan
  void GetPathParams(float distance_remaining, float curvature, ros::Time scan_time_stamp);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);
  // Controller function to determine the next time-step command velocity
  float OneDTimeOptimalControl(float v0, float distRem);
  // to transform poses 
  std::tuple<Eigen::Vector2f, float> getRelativePose(const Eigen::Vector2f initPos, float initAngle, 
    const Eigen::Vector2f endPos, float endAngle) const; 

  std::tuple<float, float, float> GetPathScoringParams(float curvature_of_turning, Eigen::Vector2f& closest_point);

  void UpdateVelocityProfile(float last_vel);

  std::tuple<float, float> getOptimalAction(float distance_remaining);

 private:

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_; 
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  // Variables updated by subscribing to server_path_params topic
  float distance_remaining_;
  float chosen_curvature_;
  ros::Time scan_time_stamp_;

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

  // navigation variables
  float length = 0.55;
  float width = 0.3;
  float wheel_base = 0.32;
  float track_width = 0.16;
  float safety_margin = 0.05; 

  Eigen::Vector2f center_of_curve;

  // Shield
  std::map<std::string, float> shield_;
  float pmax_threshold = 0.95;
  void readShieldCSV();
  float getShieldedAction(float state, float action);

};

}  // namespace navigation_client

#endif  // NAVIGATION_CLIENT_H