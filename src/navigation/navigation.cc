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
float Navigation::getMaxDistanceWithoutCollision(float curvature_of_turning, Eigen::Vector2f& closest_point) {
  float smallest_distance;
  
	// Handle case when point_cloud has no points
	if (point_cloud_.size() == 0)
		return 1; // Returning 1m. TODO : Find something better to do
	
	// Case-1 : Moving in an arc
	if (fabs(curvature_of_turning) > kEpsilon) {
		float radius_of_turning_nominal = fabs(1/curvature_of_turning);
		float direction_of_turning = (signbit(curvature_of_turning) ?  -1 : 1); //1: left turn, -1: right turn
		Eigen::Vector2f center_of_turning_left(0, radius_of_turning_nominal);
		
		// Evaluate the minimum and maximum radius of the robot swept volume
		float x_eval = radius_of_turning_nominal + 0.5*(width - track_width) + 0.5*track_width + safety_margin;
		float y_eval = 0.5*(length - wheel_base) + wheel_base + safety_margin;
		float radius_of_turning_max = sqrt(x_eval*x_eval + y_eval*y_eval);
		float radius_of_turning_min = radius_of_turning_nominal - (0.5*(width - track_width) + 0.5*track_width + safety_margin);
		float X = 0.5*wheel_base + 0.5*length + safety_margin;
		float Y = 0.5*width + safety_margin;
		
		float smallest_angular_distance = 17*M_PI/18; // TODO : Something better to avoid full circular motions

    // To compensate for the latency
    // float rcs_theta_future = (vel_sum * del_t) / radius_of_turning_nominal;
    // float rcs_x_future = radius_of_turning_nominal * sin(rcs_theta_future);
    // float rcs_y_future = radius_of_turning_nominal * (1 - cos(rcs_theta_future));
    
    float rcs_theta_future = 0.0;
    float rcs_x_future = 0.0;
    float rcs_y_future = 0.0;

		// Iterate over point cloud to find the closest point
		for (unsigned int i = 0; i < point_cloud_.size(); i++) {
			Eigen::Vector2f point_eval = transformAndEstimatePointCloud(rcs_x_future,
      rcs_y_future, rcs_theta_future, point_cloud_[i]);
			
			// Make turning right have same equations as turning left 
			point_eval.y() = direction_of_turning*point_eval.y();		
			float radius_of_point_eval = (point_eval - center_of_turning_left).norm();
			
			// Check if there exists a possibility of collision with the robot
			if (radius_of_point_eval <= radius_of_turning_max && radius_of_point_eval >= radius_of_turning_min) {
				float beta_1 = kInf;
				float beta_2 = kInf;
				
				if (fabs(X) <= radius_of_point_eval)
					beta_1 = asin(X/radius_of_point_eval); // Returns a value between 0 and pi/2
				if (fabs(radius_of_turning_nominal - Y) <= radius_of_point_eval)
					beta_2 = acos((radius_of_turning_nominal - Y)/radius_of_point_eval); // Returns a value between 0 and pi/2
				
        if (beta_2 <-1.0) 
          std::cout<<"Beta2 is -pi/2"<<"\n";
				float beta = fmin(beta_1, beta_2);
				
				if (beta != kInf) {
					float alpha = atan2(point_eval.x(), radius_of_turning_nominal - point_eval.y());
          float angular_distance = alpha - beta;
					if (angular_distance > 0) {// Checks if the point is in front of the robot 
            if (angular_distance < smallest_angular_distance) {
                closest_point = point_cloud_[i];
                // std::cout<<"Closest Point Coordinates: "<<closest_point<<"\n";
            }
						smallest_angular_distance = fmin(smallest_angular_distance, angular_distance);
          }
				}
			}
		}
		smallest_distance = radius_of_turning_nominal*smallest_angular_distance;
	}
	
	// Case-2: Moving in a straight line
	else { 
		// TODO : Complete this code
		smallest_distance = 0.5;
	}

	return smallest_distance;
}

float Navigation::OneDtoc(float v0, float distRem){
  float v1 = 0;
  if (distRem <= 0.0) {
    v1 = 0.0;
    return v1;
  }
  else {
    v1 = v0 + max_acc*del_t;
    float del_s1 = v1*del_t + 0.5*v1*v1/max_dec;
    float del_s2 = 0.5*v0*v0/max_dec;
    if (v1 <= max_vel && del_s1 < distRem) {
      return v1;
      //std::cout << "Acceleration phase" << "\n";
    }
    else if (v0 <= max_vel && del_s2 < distRem) {
      return v0;
      //std::cout << "Cruise phase" << "\n";
    }
    else {
      v1 = v0 - max_dec*del_t;
      return v1;
      //std::cout << "Deceleration phase" << "\n";
    }
  }
}

void Navigation::updateVelocityProfile(float last_vel){
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

Vector2f Navigation::transformAndEstimatePointCloud(float x, float y, float theta, Vector2f pt){
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

  // get the location of the robot with respect to its initial reference frame
  Eigen::Vector2f relPos;
  float relAngle;
  std::tie(relPos, relAngle) = getRelativePose(odom_start_loc_, odom_start_angle_, odom_loc_, odom_angle_);
  
  //adjust the distance traveled by taking into account the latency compensation
  // float distTrav = relPos.norm();
  // vel_sum = 0;
  // for(int vel_idx = 0; vel_idx < system_lat; vel_idx++){
  //   vel_sum += vel_profile[vel_idx];
  // }
  // float state = distTrav + (vel_sum) * del_t;
  
  Eigen::Vector2f closest_point;
  float distRem = 0.0;

  for (float curvature_eval = 0.9; curvature_eval >= -0.9; curvature_eval = curvature_eval - 0.2) {
    float distRem_eval = getMaxDistanceWithoutCollision(curvature_eval, closest_point);
    // std::cout << "Curvature: "<< curvature_eval << " Distance remaining: " << distRem_eval << "\n";

    if (distRem_eval > distRem) {
      drive_msg_.curvature = curvature_eval;
      distRem = distRem_eval;
    }
  }
  std::cout << "Chosen curvature:"<< drive_msg_.curvature << "Chosen distance remaining: " << distRem << "\n";

  float v0 = vel_profile[system_lat - 1];
  drive_msg_.velocity = OneDtoc(v0, distRem);
  
  // Create visualizations.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();

  visualization::DrawCross(closest_point, 0.5, 0x000000, local_viz_msg_);
  visualization::DrawRobotMargin(length, width, wheel_base, track_width, safety_margin, local_viz_msg_);

  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);

  updateVelocityProfile(drive_msg_.velocity);
  //updateLaserProfile(point_cloud_);

  
}

}  // namespace navigation
