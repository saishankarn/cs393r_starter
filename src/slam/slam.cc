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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================
 
#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2f(0, 0);
  *angle = 0;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
}



std::tuple<Eigen::Vector2f, float> ParticleFilter::MotionModel(const Eigen::Vector2f& prevLoc,
                                                                const float prevAngle,
                                                                const Eigen::Vector2f& odomLoc,
                                                                const float odomAngle,
                                                                const Eigen::Vector2f& prevOdomLoc,
                                                                const float prevOdomAngle) {
  Eigen::Rotation2Df rotOdom(-prevOdomAngle); // For transformation from Odometry to Base Link frame.
  Eigen::Vector2f deltaLoc = rotOdom.toRotationMatrix()*(odomLoc - prevOdomLoc); // Translation in Base Link frame.


  Eigen::Rotation2Df rotBase(prevAngle); // For transformation from Base Link frame to Map frame.
  Eigen::Vector2f loc = prevLoc + rotBase.toRotationMatrix()*deltaLoc; // Location in Map frame.

  float deltaAngle = odomAngle - prevOdomAngle; // Change in angle as measured by Odometry.
  
  // Accounting for non-linear scale of angles
  deltaAngle = (fabs(fabs(deltaAngle) - 2*M_PI) < fabs(deltaAngle)) ? 
                signbit(deltaAngle)*(fabs(deltaAngle) -2*M_PI) : deltaAngle;

  float angle = prevAngle + deltaAngle; // Angle in Map frame.

  return std::make_tuple(loc, angle);
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    robot_locs.push_back(Eigen::Vector2f(0.0, 0.0));
    robot_angles.push_back(0.0);
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
  
  // obtaining the latest robot pose
  Eigen::Vector2f latest_robot_loc = robot_locs.back();
  float latest_robot_angle = robot_angles.back();

  // obtaining the updated pose using motion model
  std::tie(loc, angle) = MotionModel(latest_robot_loc, latest_robot_angle,
    odom_loc, odom_angle, prev_odom_loc_, prev_odom_angle_);

  // adding the updated pose to the robot trajectory
  robot_locs.push_back(loc);
  robot_angles.push_back(angle);

  // updating the previous odometry values
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;

}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
