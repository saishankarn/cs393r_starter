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
\file    navigation_client.cc
\brief   Starter code for navigation_client.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================
#include "gflags/gflags.h" 
#include "eigen3/Eigen/Dense" 
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation_client.h"
#include "visualization/visualization.h"

# include <iostream>

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using ros::Time;
using std::string;
using std::vector;
using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
const float kInf = 1e5;
const float localGoal = 5.0;
const float dtgWeight = -0.05;
const float clWeight = 50.0;//200;

std::ofstream log_file_run_loop("log/2023-jan-28/run_loop" + std::to_string(std::time(0)) + ".csv");

} //namespace

namespace navigation_client {

Navigation_client::Navigation_client(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(5, 0),
    nav_goal_angle_(0),
    distance_remaining_(0.0),
    chosen_curvature_(0.0),
    center_of_curve(0, 0){
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  
  /* 
    READING SHIELD CSV:
      Takes a considerable amount of time. Hence ros_time commands need to be executed 
      after this function call.
  */
  readShieldCSV();

  /*
    SERVER MESSAGE QUEUE:
      The server publishes at twice the rate that the client subscribes.
      The initial size of the srv_msg_queue_ will be (2*net_lat + 1).
      Example:
        Let,
          net_lat  = 4
          time_per = 0.1
          des_del  = 0.40 (net_lat * time_per)
          server_del_t = 0.05
        
        Then, if the current time is t,
          srv_msg_queue_ = [t - 0.400, t - 0.350, t - 0.300, t - 0.250,
                            t - 0.200, t - 0.150, t - 0.100, t - 0.050, t]
        
        This means that the first information processed by the Run() loop
        has a time stamp of (t - 0.400)
  */
  for (int16_t i = 0; i < 2*net_lat + 1; i++) {
    ros::Duration d(des_del - tim_per*0.5*i); //in seconds
    srvMsgStruct srvMsg = {distance_remaining_, chosen_curvature_, ros::Time::now() - d};
    srv_msg_queue_.push(srvMsg);
  }

  /*
    LOGGING STATE-ACTION DATA
  */
  log_file_run_loop << "State, Optimal Action, Shielded Action \n";
}

void Navigation_client::readShieldCSV() {
  std::ifstream fin("data/state_action_safety_values_4_td.csv");
  if (!fin) {
    std::cout << "Error, could not open file" << std::endl;
  }
  std::string state_string;
  std::string pmax;

  // Reading data as string
  std::string line;
  while (std::getline(fin, line)) {
    std::stringstream strstrm(line);
    std::getline(strstrm, state_string, ',');
    std::getline(strstrm, pmax);
    shield_[state_string] = std::stof(pmax);
  }
  fin.close();
}

float Navigation_client::getShieldedAction(float state, float action){
  // state: distance remaining, action: velocity command
  
  // Abstracting continuous state to discrete value
  // -inf -  0.0 -> 0
  //  0.0 -  0.5 -> 1
  //  ...
  //  4.5 -   5 -> 10
  //  5   - inf -> 11
    
  int abstract_state = static_cast<int>(std::ceil(state/(0.5f)));
  abstract_state < 0 ? abstract_state = 0 : (abstract_state > 11 ? abstract_state = 11 : 0);   
  std::string key = std::to_string(abstract_state);
  
  // Abstracting continuous actions to discrete value
  // -0.50 - -0.25 -> 0
  // -0.25 -  0.00 -> 1
  //  0.00 -  0.25 -> 2
  //  0.25 -  0.50 -> 3 
  // ...
  //  0.75 -  1    -> 5
  
  for(unsigned int i = 0; i < action_queue_.size(); i++) {
    int abstract_action = static_cast<int>(std::ceil(action_queue_[i]/(0.25f))) + 1;
    abstract_action < 0 ? abstract_action = 0 : (abstract_action > 5 ? abstract_action = 5 : 0);
    key = key + '-' + std::to_string(abstract_action);    
  }

  int abstract_action = static_cast<int>(std::ceil(action/(0.25f))) + 1;
  abstract_action < 0 ? abstract_action = 0 : (abstract_action > 5 ? abstract_action = 5 : 0);
  std::string key_temp = key + '-' + std::to_string(abstract_action);
  
  if(shield_[key_temp] > pmax_threshold) {
    return action;
  }
  else {
    float pmax = -1.0;
    int pmax_i = 0;
    for(int i = 0; i <= 5 ; i++) {
      key_temp = key + '-' + std::to_string(i);
      if(shield_[key_temp] > pmax) {
        pmax = shield_[key_temp];
        pmax_i = i;
      }
    }
    
    // pmax_i : [0,..., 5]
    return std::max(0.0, pmax_i * 0.25 - 0.250);
  }
}

std::tuple<Eigen::Vector2f, float> Navigation_client::getRelativePose(
  const Eigen::Vector2f initPos, float initAngle, 
  const Eigen::Vector2f endPos, float endAngle) const {

  Eigen::Rotation2Df rotMat(initAngle);
  Eigen::Vector2f pos = rotMat.toRotationMatrix().inverse()*(endPos - initPos);
  float angle = endAngle - initAngle;

  return std::make_tuple(pos, angle);
}

/**
 * @brief One-D Time Optimal Control
 * 
 * @param v0 Previous velocity  
 * @param distance_remaining Distance at which velocity is to be zero
 * @return float - Desired current velocity
 */
float Navigation_client::OneDTimeOptimalControl(float v0, float distance_remaining){
  float v1 = 0;
  if (distance_remaining <= 0.0) {
    v1 = 0.0;
    return v1;
  }
  else {
    v1 = v0 + max_acc*del_t;
    float del_s1 = v1*del_t + 0.5*v1*v1/max_dec;
    float del_s2 = 0.5*v0*v0/max_dec;
    if (v1 <= max_vel && del_s1 < distance_remaining) {
      return v1;
    }
    else if (v0 <= max_vel && del_s2 < distance_remaining) {
      return v0;
    }
    else {
      v1 = v0 - max_dec*del_t;
      v1 = std::max(v1, (float )0);
      return v1;
    }
  }
}

void Navigation_client::UpdateVelocityProfile(float last_vel){
  for(int vel_idx = 0; vel_idx < system_lat-1; vel_idx++){
    vel_profile[vel_idx] = vel_profile[vel_idx+1];  
  }
  vel_profile[system_lat-1] = last_vel;
}

void Navigation_client::UpdateActionQueue(float last_action){
  for(int act_idx = 0; act_idx < total_lat_-1; act_idx++){
    action_queue_[act_idx] = action_queue_[act_idx+1];  
  }
  action_queue_[total_lat_-1] = last_action;
}

void Navigation_client::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation_client::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation_client::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;

  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;

    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation_client::QueueSrvMsg(srvMsgStruct srvMsg) {
  srv_msg_queue_.push(srvMsg);
}

float Navigation_client::CompensateSystemDelay(float distance_remaining) {
  
  /**
   * Time optimal control with compensation for system latency
   * 
   * This is achieved by adjust the distance traveled by taking into account the actuation latency compensation
   * System Latency = Sensor Latency + Actuation Latency 
   * This controller assumes static obstacles in the environment
   * 
   */
  float vel_sum = 0.0f;
  for(int vel_idx = 0; vel_idx < system_lat; vel_idx++){
    vel_sum += vel_profile[vel_idx];
  }
  float dis_rem_delay_compensated = distance_remaining - (vel_sum) * del_t;
  
  return dis_rem_delay_compensated;
}

/**
 * @brief This function gets called 20 times a second to form the control loop.
 * 
 */
void Navigation_client::Run() {

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);
 
  // If odometry has not been initialized, we can't do anything.
  // if (!odom_initialized_) return;

  // Get the location of the robot with respect to its initial reference frame.
  Eigen::Vector2f relPos;
  float relAngle;
  std::tie(relPos, relAngle) = getRelativePose(odom_start_loc_, odom_start_angle_, odom_loc_, odom_angle_);
 
  // Dequeue server commands
  ros::Time scan_time_stamp;
  if (!srv_msg_queue_.empty()) {
    srvMsgStruct srvMsg = srv_msg_queue_.front();;
    do {
      distance_remaining_ = srvMsg.distance_remaining;
      chosen_curvature_   = srvMsg.curvature;
      scan_time_stamp     = srvMsg.scan_time_stamp;
      srv_msg_queue_.pop();
      if (srv_msg_queue_.empty())
        break;
      else {
        srvMsg = srv_msg_queue_.front();
      }
    }
    while (((ros::Time::now() - srvMsg.scan_time_stamp).toSec()) > des_del);
  }
  else {
    std::cout << "Server msg queue empty \n";
    scan_time_stamp    = ros::Time::now();
  }
  //std::cout << "Time delay set for system: " <<  (ros::Time::now() - scan_time_stamp)*1000 << "\n";
  
  /*
    OBTAIN OPTIMAL ACTION
      If no shield is present use default system delay compensation.
      Else shield for total delay (system delay + network delay) 
  */
  
  // float dis_rem_delay_compensated = CompensateSystemDelay(distance_remaining_);
  // float opt_action = OneDTimeOptimalControl(vel_profile[system_lat - 1], dis_rem_delay_compensated);
  float opt_action = OneDTimeOptimalControl(vel_profile[system_lat - 1], distance_remaining_);
  std::cout << "Dis rem: " << distance_remaining_ << std::endl;
  
  // OBTAIN SHIELDED ACTION
  float shielded_action = getShieldedAction(distance_remaining_, opt_action);
  // float shielded_action = opt_action;
  
  // Update drive message
  drive_msg_.curvature = chosen_curvature_;
  drive_msg_.velocity  = shielded_action;
  
  // Update velocity profile
  UpdateVelocityProfile(drive_msg_.velocity);
  UpdateActionQueue(drive_msg_.velocity);

  // Create visualizations.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  visualization::DrawRobotMargin(length, width, wheel_base, track_width, safety_margin, local_viz_msg_);
  //visualization::DrawCross(collision_point, 0.5, 0x000000, local_viz_msg_);

  // LOG DATA
  log_file_run_loop << distance_remaining_ << ','
                    << opt_action << ','
                    << shielded_action << '\n';

  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
  
}

}  // namespace navigation_client
