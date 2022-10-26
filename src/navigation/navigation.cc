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
\file    navigation.cc
\brief   Starter code for navigation.
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
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
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
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(5, 0),
    nav_goal_angle_(0),
    center_of_curve(0, 0){
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  readShieldCSV();
}

void Navigation::readShieldCSV() {
  std::ifstream fin("data/own_state_action_values_4_td_csv.csv");
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

float Navigation::getShieldedAction(float state, float action){
  // state: distance remaining, action: velocity command
  
  // Abstracting continuous state to discrete value
  // 0.0 - 0.2 -> 0
  // 0.2 - 0.4 -> 1
  // ...
  // 9.8 - 10 -> 49
  // 10  - 20 -> 50
  int abstract_state = static_cast<int>(std::lround(state/(0.2f)));
  abstract_state < 0 ? abstract_state = 0 : (abstract_state > 50 ? abstract_state = 50 : 0);   
  std::string key = std::to_string(abstract_state);
  
  // Abstracting continuous actions to discrete value
  // -1.0 - -0.8 -> 0
  // ...
  // -0.2 -  0.0 -> 4
  //  0.0 -  0.2 -> 5
  //  0.8 -  1.0 -> 9 
  for(unsigned int i = 0; i < vel_profile.size(); i++) {
    int abstract_action = static_cast<int>(std::lround(vel_profile[i]/(0.2f))) + 5;
    abstract_action < 0 ? abstract_action = 0 : (abstract_action > 9 ? abstract_action = 9 : 0);
    key = key + '-' + std::to_string(abstract_action);       
  }

  int abstract_action = static_cast<int>(std::lround(action/(0.2f))) + 5;
  abstract_action < 0 ? abstract_action = 0 : (abstract_action > 9 ? abstract_action = 9 : 0);
  std::string key_temp = key + '-' + std::to_string(abstract_action);
  // std::cout << key_temp << "\n";
  // std::cout << shield_[key_temp] << "\n";
  if(shield_[key_temp] > pmax_threshold) {
    return action;
  }
  else {
    float pmax = -1.0;
    int pmax_i = 5;
    for(int i = 5; i < 10 ; i++) {
      key_temp = key + '-' + std::to_string(i);
      // std::cout << key_temp << "\n";
      std::cout << key_temp << ": " << shield_[key_temp] << "\n";
      if(shield_[key_temp] > pmax) {
        pmax = shield_[key_temp];
        pmax_i = i;
      }
    }
    
    return std::max(0.0, std::max(5, pmax_i)*0.2 - 1.0);
  }
}

std::tuple<Eigen::Vector2f, float> Navigation::getRelativePose(
  const Eigen::Vector2f initPos, float initAngle, 
  const Eigen::Vector2f endPos, float endAngle) const {

  Eigen::Rotation2Df rotMat(initAngle);
  Eigen::Vector2f pos = rotMat.toRotationMatrix().inverse()*(endPos - initPos);
  float angle = endAngle - initAngle;

  return std::make_tuple(pos, angle);
}

// Inputs:  Curvature of turning
// Outputs: Distance remaining
std::tuple<float, float, float> Navigation::GetPathScoringParams(float curvature_of_turning, Eigen::Vector2f& collision_point) {
  float freePathLength = 5.0;
  float distanceToGoal = 5.0;
  float clearance = 5.0;
  
  // Handle case when point_cloud has no points
  if (point_cloud_.size() == 0)
    return std::make_tuple(freePathLength, distanceToGoal, clearance);
  
  // Case-1 : Moving in an arc
  if (fabs(curvature_of_turning) > kEpsilon) {
    float radius_of_turning_nominal = fabs(1/curvature_of_turning); // radius of turning of the robot origin
    float direction_of_turning = (signbit(curvature_of_turning) ?  -1 : 1); //1: left turn, -1: right turn
    Eigen::Vector2f center_of_turning(0, radius_of_turning_nominal);
    
    // Evaluate the minimum and maximum radius of the robot swept volume
    float x_eval = radius_of_turning_nominal + 0.5*(width - track_width) + 0.5*track_width + safety_margin;
    float y_eval = 0.5*(length - wheel_base) + wheel_base + safety_margin;
    float radius_of_turning_max = sqrt(x_eval*x_eval + y_eval*y_eval); // front right corner of robot margin boundary 
    float radius_of_turning_min = radius_of_turning_nominal - (0.5*(width - track_width) + 0.5*track_width + safety_margin); // side left point of robot margin boundary
    float X = 0.5*wheel_base + 0.5*length + safety_margin;
    float Y = 0.5*width + safety_margin;
    
    float smallest_angular_distance = 17*M_PI/18; // TODO : Something better to avoid full circular motions
    //float alpha_min = 17*M_PI/18;

    // Latency compensation grouped together and pushed to later
    // float rcs_theta_future = (vel_profile[0] * del_t) / radius_of_turning_nominal;
    // float rcs_x_future = radius_of_turning_nominal * sin(rcs_theta_future);
    // float rcs_y_future = radius_of_turning_nominal * (1 - cos(rcs_theta_future));
    
    float rcs_theta_future = 0.0;
    float rcs_x_future = 0.0;
    float rcs_y_future = 0.0;

    // Finding the free path length.
    for (unsigned int i = 0; i < point_cloud_.size(); i++) {
      Eigen::Vector2f point_candidate = TransformAndEstimatePointCloud(rcs_x_future,
      rcs_y_future, rcs_theta_future, point_cloud_[i]);
      
      // Make turning right have same equations as turning left 
      point_candidate.y() = direction_of_turning*point_candidate.y();   
      float radius_of_point_candidate = (point_candidate - center_of_turning).norm();
      
      // Check if there exists a possibility of collision with the robot
      if (radius_of_point_candidate <= radius_of_turning_max && radius_of_point_candidate >= radius_of_turning_min) {
        float beta_1 = kInf;
        float beta_2 = kInf;
        
        if (fabs(X) <= radius_of_point_candidate) {
          beta_1 = asin(X/radius_of_point_candidate); // Returns a value between 0 and pi/2
        }
        if (fabs(radius_of_turning_nominal - Y) <= radius_of_point_candidate)
          beta_2 = acos((radius_of_turning_nominal - Y)/radius_of_point_candidate); // Returns a value between 0 and pi/2
        
        float beta = fmin(beta_1, beta_2);
        
        if (beta != kInf) {
          float alpha = atan2(point_candidate.x(), radius_of_turning_nominal - point_candidate.y());
          float angular_distance = alpha - beta;
          if (angular_distance > 0) {// Checks if the point is in front of the robot 
            if (angular_distance < smallest_angular_distance) {
                collision_point = point_cloud_[i]; // just for visualization
                smallest_angular_distance = angular_distance;
                //alpha_min = alpha;
            }
          }
        }
        else {;}// assuming this case never happens. need to prove mathematically
      }
      else {;}// smallest_angular_distance is the previous computed value
    }
    freePathLength = radius_of_turning_nominal*smallest_angular_distance;
    freePathLength = std::min(freePathLength, float(5.0));

    // Finding the clearance.
    for (unsigned int i = 0; i < point_cloud_.size(); i++) {// TODO : Alternate to iterating through the point cloud again
      Eigen::Vector2f point_candidate = TransformAndEstimatePointCloud(rcs_x_future,
      rcs_y_future, rcs_theta_future, point_cloud_[i]);
      
      // Make turning right have same equations as turning left 
      point_candidate.y() = direction_of_turning*point_candidate.y();   
      float radius_of_point_candidate = (point_candidate - center_of_turning).norm();

      float point_candidate_angular_distance = atan2(point_candidate.x(), radius_of_turning_nominal - point_candidate.y());
      // std::cout << "alpha_min: " << alpha_min << "\n";
      //if ((point_candidate_angular_distance < alpha_min) && (point_candidate_angular_distance > 0.0)) {
      if (point_candidate_angular_distance > -M_PI/6) {
        if (radius_of_turning_min - radius_of_point_candidate > 0.0) {
          clearance = std::min(clearance, radius_of_turning_min - radius_of_point_candidate);
        }
        if (radius_of_point_candidate - radius_of_turning_max > 0.0) {
          clearance = std::min(clearance, radius_of_point_candidate - radius_of_turning_max);
        }
      }
    }

    // Finding the distance to goal.
    distanceToGoal = (Vector2f(localGoal, 0) - Vector2f(0, radius_of_turning_nominal)).norm() - radius_of_turning_nominal;

  }
  
  // Case-2: Moving in a straight line
  else { 
    // TODO: need to include safety margin
    freePathLength = 20.0;
    clearance = 10.0;

    float sweptBound = width / 2 + safety_margin;

    // Compensating for the sensor latency
    float rcs_theta_future = 0;
    float rcs_x_future = 0;
    // float rcs_x_future = vel_profile[0] * del_t;
    float rcs_y_future = 0;

    for (unsigned int i = 0; i < point_cloud_.size(); i++) {
      Eigen::Vector2f point_candidate = TransformAndEstimatePointCloud(rcs_x_future,
      rcs_y_future, rcs_theta_future, point_cloud_[i]);

      if (point_candidate.x() > 0){
        if (fabs(point_candidate.y()) < sweptBound){
          float pathLength = point_candidate.x();
          if (pathLength < freePathLength) {
            freePathLength = pathLength;
            collision_point = point_cloud_[i];
          }
        }
        else{ 
        clearance = std::min(clearance, fabs(point_candidate.y()) - sweptBound);
        }
      }
    }
    std::cout << "Free path length: " << freePathLength << "\n";
    // Including the safety margin in the free path length
    freePathLength = freePathLength - (0.5*wheel_base + 0.5*length + safety_margin) ;
    distanceToGoal = std::max(0.0f, localGoal - freePathLength);
  }
  return std::make_tuple(freePathLength, distanceToGoal, clearance);
}

float Navigation::OneDTimeOptimalControl(float v0, float distance_remaining){
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
      // std::cout << "Acceleration phase" << "\n";
      return v1;
    }
    else if (v0 <= max_vel && del_s2 < distance_remaining) {
      // std::cout << "Cruise phase" << "\n";
      return v0;
    }
    else {
      v1 = v0 - max_dec*del_t;
      v1 = std::max(v1, (float )0);
      // std::cout << "Deceleration phase" << "\n";
      return v1;
    }
  }
}

void Navigation::UpdateVelocityProfile(float last_vel){
  for(int vel_idx = 0; vel_idx < system_lat-1; vel_idx++){
    vel_profile[vel_idx] = vel_profile[vel_idx+1];  
  }
  vel_profile[system_lat-1] = last_vel;
}


void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  // std::cout << "Robot Velocity: " << vel << "\n"; 

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

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;    

}

Vector2f Navigation::TransformAndEstimatePointCloud(float x, float y, float theta, Vector2f pt){
  Eigen::Matrix3f T;
  T << cos(theta), -sin(theta), x, 
       sin(theta), cos(theta), y, 
       0, 0, 1;
  T = T.inverse();
  Eigen::Vector3f pt3(pt[0], pt[1], 1);
  pt3 = T*pt3;
  return Vector2f(pt3[0], pt3[1]);
}

//vector<bool> Navigation::DetectCollisionPts(float curvature)

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);
 
  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // Get the location of the robot with respect to its initial reference frame.
  Eigen::Vector2f relPos;
  float relAngle;
  std::tie(relPos, relAngle) = getRelativePose(odom_start_loc_, odom_start_angle_, odom_loc_, odom_angle_);
  
  // Adjust the distance traveled by taking into account the actuation latency compensation.
  // System Latency = Sensor Latency + Actuation Latency 
  vel_sum = 0;
  for(int vel_idx = 0; vel_idx < system_lat; vel_idx++){
    vel_sum += vel_profile[vel_idx];
  }
  
  // Obstacle avoidance.
  Eigen::Vector2f collision_point; // to visualize the first point of collision with an obstacle given a path
  Eigen::Vector2f collision_point_candidate;
  float distance_remaining = 0.0; // setting a minimum value of zero
  
  // float best_score = -kInf;
  // for (float curvature_candidate = 1.05; curvature_candidate >= -1.05; curvature_candidate = curvature_candidate - 0.1) {
  //   float freePathLengthCandidate;
  //   float distanceToGoalCandidate;
  //   float clearanceCandidate;
  //   std::tie(freePathLengthCandidate, distanceToGoalCandidate, clearanceCandidate) = GetPathScoringParams(curvature_candidate, collision_point_candidate);
  //   float score = freePathLengthCandidate + dtgWeight * distanceToGoalCandidate + clWeight * clearanceCandidate;
    
  //   // Visualizing candidate arcs and correspoing distances
  //   visualization::DrawPath(curvature_candidate, freePathLengthCandidate, 0xFFA500, local_viz_msg_);
  //   // float radius_of_turning_min = fabs(1/curvature_candidate) - (0.5*(width - track_width) + 0.5*track_width + safety_margin);
  //   // visualization::DrawPathOption(curvature_candidate, radius_of_turning_min*freePathLengthCandidate, 0.5*width+safety_margin, local_viz_msg_);
  //   // std::cout << "C: "<< curvature_candidate << ", FPL: " << freePathLengthCandidate << ", DTG: " 
  //             // << dtgWeight*distanceToGoalCandidate << ", Cl: " << clWeight*clearanceCandidate << ", S: " <<score << "\n";

  //   // Choosing the arc/line with the best score
  //   if (score > best_score) {
  //     best_score = score;
  //     distance_remaining = freePathLengthCandidate;
  //     collision_point = collision_point_candidate;
  //     drive_msg_.curvature = curvature_candidate; // choosing curvature with best score
  //   }
  // }
  
  // std::cout << "Chosen curvature: "<< drive_msg_.curvature << " Chosen distance remaining: " << distance_remaining << "\n";

  // STRAIGHT LINE MOTION ONLY
  float curvature_candidate = 0.0;
  float freePathLengthCandidate;
  float distanceToGoalCandidate;
  float clearanceCandidate;
  std::tie(freePathLengthCandidate, distanceToGoalCandidate, clearanceCandidate) = GetPathScoringParams(curvature_candidate, collision_point_candidate);
  distance_remaining = freePathLengthCandidate;
  collision_point = collision_point_candidate;
  drive_msg_.curvature = curvature_candidate;

  // NAIVE LATENCY COMPENSATION
  // Works well for static obstacles in the environment
  float v0 = vel_profile[system_lat - 1];
  float dis_rem_delay_compensated = distance_remaining - (vel_sum) * del_t;
  std::cout << "Distance remaining: " << distance_remaining << "; Delay compensated distance remaining" 
  << dis_rem_delay_compensated << "\n";

  // Time optimal control.
  float opt_action = OneDTimeOptimalControl(v0, dis_rem_delay_compensated);
  drive_msg_.velocity = opt_action;
  
  // Shielding here
  float shielded_action = getShieldedAction(distance_remaining, opt_action);
  std::cout << "OA: " << opt_action << ", SA: " << shielded_action << "\n";
  // drive_msg_.velocity = shielded_action;
  
  // Update velocity profile
  UpdateVelocityProfile(drive_msg_.velocity);

  // Create visualizations.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  visualization::DrawRobotMargin(length, width, wheel_base, track_width, safety_margin, local_viz_msg_);
  visualization::DrawCross(collision_point, 0.5, 0x000000, local_viz_msg_);
  

  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
  
}

}  // namespace navigation