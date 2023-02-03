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
\file    serverside.cc 
\brief   Starter code for serverside.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================
#include "gflags/gflags.h" 
#include "eigen3/Eigen/Dense" 
#include "eigen3/Eigen/Geometry"
#include "glog/logging.h"
#include "ros/ros.h" 
#include "shared/math/math_util.h"
#include "shared/util/timer.h" 
#include "shared/ros/ros_helpers.h"
#include "serverside.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PointStamped.h"

using Eigen::Vector2f;
using std::string;
using std::vector;
//using geometry_msgs::PointStamped;
 
using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher server_pub_;
//ros::Publisher depth_pub_; 
//ros::Publisher drive_pub_;
//ros::Publisher viz_pub_;

//VisualizationMsg local_viz_msg_;
//VisualizationMsg global_viz_msg_;
geometry_msgs::PointStamped server_msg_;
//std_msgs::Float32 depth_msg_;

// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
const float kInf = 1e5;
const float localGoal = 5.0;
const float dtgWeight = -0.05;
const float clWeight = 50.0;//200;
} //namespace

namespace serverside {

Serverside::Serverside(const string& map_file, ros::NodeHandle* n) :
    center_of_curve(0, 0){
  //depth_pub_ = n->advertise<std_msgs::Float32>(
  //    "serverside_depth_info", 1);
  server_pub_ = n->advertise<geometry_msgs::PointStamped>("server_path_params", 1);
  //viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  //local_viz_msg_ = visualization::NewVisualizationMessage(
  //    "base_link", "navigation_local");
  //global_viz_msg_ = visualization::NewVisualizationMessage(
  //    "map", "navigation_global");
  InitRosHeader("base_link", &server_msg_.header);
  //readShieldCSV();
}


// Inputs:  Curvature of turning
// Outputs: Distance remaining
std::tuple<float, float, float> Serverside::GetPathScoringParams(float curvature_of_turning, Eigen::Vector2f& collision_point) {
  float freePathLength = 5.0;
  float distanceToGoal = 5.0;
  float clearance = 5.0;
  //std::cout << "In GetPathScoringParams function" << "\n";
  
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
      std::cout << i << "\n";
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
    // Including the safety margin in the free path length
    freePathLength = freePathLength - (0.5*wheel_base + 0.5*length + safety_margin) ;
    distanceToGoal = std::max(0.0f, localGoal - freePathLength);
    //std::cout << freePathLength << "\n";
  }
  return std::make_tuple(freePathLength, distanceToGoal, clearance);
}

void Serverside::ObservePointCloud(const vector<Vector2f>& cloud,
                                   ros::Time time) {
  //std::cout << "time difference : " << time - point_cloud_time_stamp_ << "\n";
  //point_cloud_ = cloud; 
  //point_cloud_time_stamp_ = time;   

  point_cloud_stack_.push_back(cloud);
  point_cloud_time_stamp_stack_.push_back(time);
  std::cout << "In the callback function, the time stamp being added is " << time << "\n";
  std::cout << "size of point cloud stack is " << size(point_cloud_stack_) << "\n";

}

void Serverside::PopulateServersideBuffers() {

  std::cout << "The current value of choice index is " << choice_index << "\n";

  if (size(point_cloud_stack_) > 0) {
    int loc = choice_index;
    for (unsigned int pc_idx = 0; pc_idx < point_cloud_stack_.size(); pc_idx++) { 
      loc = choice_index + pc_idx;
      if (loc % choose_after == 0) {
        //std::cout << size(point_cloud_stack_[pc_idx]) << "\n";
        serverside_point_cloud_buffer_.push_back(point_cloud_stack_[pc_idx]);
        serverside_point_cloud_time_stamp_buffer_.push_back(point_cloud_time_stamp_stack_[pc_idx]);
      }
    }
    choice_index = loc % choose_after + 1;
    point_cloud_stack_.clear();
    point_cloud_time_stamp_stack_.clear(); 
    //std::cout << "The modified value of choice index is " << choice_index << "\n";
  } 

  if (size(serverside_point_cloud_buffer_) > 20){
    serverside_point_cloud_buffer_.clear();
    serverside_point_cloud_time_stamp_buffer_.clear(); 
  }

}

Vector2f Serverside::TransformAndEstimatePointCloud(float x, float y, float theta, Vector2f pt){
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

void Serverside::Run() {
  // This function gets called 20 times a second to form the control loop.
  // get the required point cloud from the point cloud stack
  std::cout << "The size of the serverside point cloud buffer is " << size(serverside_point_cloud_buffer_) << "\n";
  if (size(serverside_point_cloud_buffer_) > 0) {
    point_cloud_ = serverside_point_cloud_buffer_[0];
    std::cout << "point cloud size " << size(point_cloud_) << "\n"; 
    point_cloud_time_stamp_ = serverside_point_cloud_time_stamp_buffer_[0];
    serverside_point_cloud_buffer_.erase(serverside_point_cloud_buffer_.begin());
    serverside_point_cloud_time_stamp_buffer_.erase(serverside_point_cloud_time_stamp_buffer_.begin());

    std::cout << "the time stamp being sent is " << point_cloud_time_stamp_ << "\n";

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
    //std::cout << "right before the GetPathScoringParams" << "\n";
    //std::cout << "size of point cloud" << size(point_cloud_) << "\n";
    std::tie(freePathLengthCandidate, distanceToGoalCandidate, clearanceCandidate) = GetPathScoringParams(curvature_candidate, collision_point_candidate);
    //std::cout << "the distance remaning" << freePathLengthCandidate << "\n";
    distance_remaining = freePathLengthCandidate;
    collision_point = collision_point_candidate;
    std::cout << "distance remaining" << distance_remaining << "\n";

    server_msg_.point.x = distance_remaining;
    server_msg_.point.y = curvature_candidate;
    server_msg_.point.z = 0.0;


    // Create visualizations.
    //local_viz_msg_.header.stamp = ros::Time::now();
    //global_viz_msg_.header.stamp = ros::Time::now();
    //depth_msg_.header.stamp = ros::Time::now();
    //visualization::DrawRobotMargin(length, width, wheel_base, track_width, safety_margin, local_viz_msg_);
    //visualization::DrawCross(collision_point, 0.5, 0x000000, local_viz_msg_);
    server_msg_.header.stamp = point_cloud_time_stamp_;
    //std::cout << "-------------------------" << "\n";
    //std::cout << "point cloud time stamp" << point_cloud_time_stamp_ << "\n";
    //std::cout << "server msg time stamp" << server_msg_.header.stamp << "\n";

    // Publish messages.
    //viz_pub_.publish(local_viz_msg_);
    //viz_pub_.publish(global_viz_msg_);
    server_pub_.publish(server_msg_);
  }
}

}  // namespace serverside
