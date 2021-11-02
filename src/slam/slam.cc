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
#include <math.h>
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

#include<opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

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

DEFINE_double(map_resolution, 0.1, "Rasterized cost map resolution");
DEFINE_double(sensor_std, 0.15, "standard deviation of sensor");

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

void SLAM::SaveRasterizedCostMap(Eigen::MatrixXf& rasterized_cost){
  rasterized_cost = (rasterized_cost.array() - rasterized_cost.maxCoeff()).exp();
  //cout << "the minimum value is : " << "  " << rasterized_cost.minCoeff();
  //cout << "the maximum value is : " << "  " << rasterized_cost.maxCoeff();
  //cv::Mat mat(600,600,CV_32FC1);
  cv::Mat mat = cv::Mat(600, 600, CV_32F);
  //cout << mat;
  //Eigen::Map<MatrixXf> rasterized_cost(mat.data());
}

void SLAM::GetObservationLikelihood(vector<float>& log_likelihood_list,
                                    Eigen::MatrixXf& rasterized_cost,
                                    vector<Vector2f> point_cloud_,
                                    float range_min,
                                    float range_max,
                                    float angle_min,
                                    float angle_max){
  // returns the observation likelihood scores of a given point_cloud_ using the rasterized_cost
  int dim = int(2 * range_max / FLAGS_map_resolution); // 600
  for(int pc_idx = 0; pc_idx < int(point_cloud_.size()); pc_idx++){
    Vector2f pt = point_cloud_[pc_idx];
    Vector2f grid_pt(int((range_max - pt[0]) / FLAGS_map_resolution),
                     int((range_max - pt[1]) / FLAGS_map_resolution));
    if (grid_pt[0] >= 0 && grid_pt[0] <= dim){
      if (grid_pt[1] >= 0 && grid_pt[1] <= dim){
        log_likelihood_list.push_back(rasterized_cost(grid_pt[0], grid_pt[1]));
        continue;
      }
    }
    log_likelihood_list.push_back(0.0); 
  }
}

void SLAM::GetRasterizedCost(const vector<float>& ranges, 
                             Eigen::MatrixXf& rasterized_cost,
                             float range_min,
                             float range_max,
                             float angle_min,
                             float angle_max){
  // Takes lidar scan as input and returns the cost array as ouput 
  
  // 1. convert the lidar scan from ranges to (x, y) coordinates 
  const Vector2f kLaserLoc(0.2, 0);
  vector<Vector2f> point_cloud_;
  float angle_increment = (angle_max - angle_min) / ranges.size();
  for(int ranges_idx = 0; ranges_idx < int(ranges.size()); ranges_idx++){
    Vector2f v(0, 0);
    v[0] = ranges[ranges_idx] * cos(angle_min + ranges_idx * angle_increment);
    v[1] = ranges[ranges_idx] * sin(angle_min + ranges_idx * angle_increment);
    point_cloud_.push_back(v + kLaserLoc);
  }
  
  // 2. calculating the sum log-likelihood scores
  int dim = int(2 * range_max / FLAGS_map_resolution); // 600
  vector<float> log_likelihood_list;
  for(int row_idx = 0; row_idx < dim; row_idx++){
    for(int col_idx = 0; col_idx < dim; col_idx++){
      Vector2f grid_pt(range_max - row_idx * FLAGS_map_resolution, 
                       range_max - col_idx * FLAGS_map_resolution);
      float sum_log_likelihood = 0.0;
      for(int ranges_idx = 0; ranges_idx < int(ranges.size()); ranges_idx++){
        Vector2f lidar_pt = point_cloud_[ranges_idx];
        float dist = (grid_pt - lidar_pt).norm();
        float log_likelihood = - (pow(dist, 2) / pow(FLAGS_sensor_std,2));
        sum_log_likelihood += log_likelihood;
      }
      log_likelihood_list.push_back(sum_log_likelihood);
    }
  }
  rasterized_cost = Eigen::MatrixXf::Map(&log_likelihood_list[0], dim, dim);
  SaveRasterizedCostMap(rasterized_cost);
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.

  Eigen::MatrixXf rasterized_cost;
  GetRasterizedCost(ranges, rasterized_cost, range_min, range_max, angle_min, angle_max);
  
}



std::tuple<Eigen::Vector2f, float> SLAM::MotionModel(const Eigen::Vector2f& prevLoc,
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
  Eigen::Vector2f loc;
  float angle;
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
