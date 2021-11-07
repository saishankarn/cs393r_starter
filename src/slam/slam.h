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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================
 
#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_; 
  size_t time_stamp_ = 0;


  // custom slam variables 
  int counter = 0;
  Eigen::Vector2f curr_robot_loc_;
  float curr_robot_angle_;
  std::vector<Eigen::Vector2f> robot_locs_;
  std::vector<float> robot_angles_;
  std::vector<Eigen::MatrixXf> rasterized_costs;
  std::vector<std::vector<Eigen::Vector2f>> point_clouds;

  // custom slam functions 
  Eigen::MatrixXf GetRasterizedCost(const std::vector<Eigen::Vector2f>& point_cloud_,
                                    const std::vector<float>& ranges,
                                    float range_min,
                                    float range_max,
                                    float angle_min,
                                    float angle_max);

  std::vector<Eigen::Vector2f> GetPointCloud(const std::vector<float>& ranges,
                                    float range_min,
                                    float range_max,
                                    float angle_min,
                                    float angle_max);

  Eigen::Vector2f TransformAndEstimatePointCloud(float x, float y, float theta, Eigen::Vector2f pt);

  float GetObservationLikelihood(Eigen::MatrixXf& rasterized_cost,
                                std::vector<Eigen::Vector2f> point_cloud_,
                                float range_max);
    
  std::tuple<Eigen::Vector2f, float> GetMostLikelyPose(const Eigen::Vector2f& currSLAMPoseLoc,                                                           
                                                           const float& currSLAMPoseAngle,
                                                           const Eigen::Vector2f& prevSLAMPoseLoc,
                                                           const float& prevSLAMPoseAngle,
                                                           Eigen::MatrixXf& rasterized_cost,
                                                           const std::vector<Eigen::Vector2f>& point_cloud_,
                                                           const float& range_max);

  std::tuple<Eigen::Vector2f, float> DeterministicMotionModel(const Eigen::Vector2f& prevLoc,
                                                 const float prevAngle,
                                                 const Eigen::Vector2f& odomLoc,
                                                 const float odomAngle,
                                                 const Eigen::Vector2f& prevOdomLoc,
                                                 const float prevOdomAngle);
};
}  // namespace slam

#endif   // SRC_SLAM_H_
