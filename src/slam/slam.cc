// Jai Shri Ram
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

#include <stdio.h>
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
#include "shared/math/statistics.h"
#include "shared/util/timer.h"
#include "visualization/CImg.h"

#include "slam.h"

#include "vector_map/vector_map.h"
#include <string>
//#include<opencv2/highgui/highgui.hpp>
//#include <opencv2/core/eigen.hpp>

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
using cimg_library::CImg;
using cimg_library::CImgDisplay;

DEFINE_double(map_resolution, 0.02, "Rasterized cost map resolution. Width of cell in meters");
DEFINE_double(sensor_std, 0.01, "standard deviation of sensor");
DEFINE_double(map_range, 5.0, "half map range");

namespace slam {

DEFINE_double(gridDelX, 0.02, "1 cm");
DEFINE_double(gridDelY, 0.02, "1 cm");
DEFINE_double(gridDelQ, math_util::DegToRad(1.0), "1 deg");
DEFINE_double(succ_trans_dist, 0.5, "50 cm");
DEFINE_double(succ_ang_dist, math_util::DegToRad(10.0), "45 deg");
DEFINE_double(std_k1, 0.2, "Translation dependence on translation standard deviation");
DEFINE_double(std_k2, 0.2, "Rotation dependence on translation standard deviation");
DEFINE_double(std_k3, 0.5, "Translation dependence on rotation standard deviation");
DEFINE_double(std_k4, 1.0, "Rotation dependence on rotation standard deviation");

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    curr_robot_loc_(0.0, 0.0),
    curr_robot_angle_(0.0) {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = curr_robot_loc_;
  *angle = curr_robot_angle_;
}

float SLAM::GetObservationLikelihood(Eigen::MatrixXf& rasterized_cost,
                                    vector<Vector2f> point_cloud_,
                                    float range_max){
  // Returns the observation likelihood scores of a given point_cloud_ using the rasterized_cost.
  float obs_log_likelihood = 0.0;
  int dim = rasterized_cost.cols();
  float minXorY =  -((dim - 1)/2)*FLAGS_map_resolution;
  for(int pc_idx = 0; pc_idx < int(point_cloud_.size()); pc_idx++){
    Vector2f pt = point_cloud_[pc_idx];
    Vector2f grid_pt(std::round((pt[1] - minXorY) / FLAGS_map_resolution),
                     std::round((pt[0] - minXorY) / FLAGS_map_resolution));
    if (grid_pt[0] >= 0 && grid_pt[0] < dim){
      if (grid_pt[1] >= 0 && grid_pt[1] < dim){
        obs_log_likelihood += std::max(rasterized_cost(grid_pt[0], grid_pt[1]), (float)-80.0);
      }
    }
  }
  obs_log_likelihood = obs_log_likelihood/800;
  return obs_log_likelihood;
}

std::vector<Vector2f> SLAM::GetPointCloud(const vector<float>& ranges,
                                    float range_min,
                                    float range_max,
                                    float angle_min,
                                    float angle_max){
  // Converts the lidar range scans to point cloud (x, y) format.
  const Vector2f kLaserLoc(0.2, 0);
  std::vector<Vector2f> point_cloud_;
  float angle_increment = (angle_max - angle_min) / ranges.size();
  for(int ranges_idx = 0; ranges_idx < int(ranges.size()); ranges_idx++){
    Vector2f v(0, 0);
    v[0] = ranges[ranges_idx] * cos(angle_min + ranges_idx * angle_increment);
    v[1] = ranges[ranges_idx] * sin(angle_min + ranges_idx * angle_increment);
    point_cloud_.push_back(v + kLaserLoc);
  }
  return point_cloud_;
}

Eigen::MatrixXf SLAM::GetRasterizedCost(const std::vector<Vector2f>& point_cloud_,
                                        const std::vector<float>& ranges,
                                        float range_min,
                                        float range_max,
                                        float angle_min,
                                        float angle_max){
  // Takes lidar scan as input and returns the cost array as ouput 
  
  // 1. convert the lidar scan from ranges to (x, y) coordinates 
  
  // 2. calculating the sum log-likelihood scores
  int dim = floor(2 * FLAGS_map_range / FLAGS_map_resolution) + 1; // 600
  float log_pdf_value;
  float minXorY =  -((dim - 1)/2)*FLAGS_map_resolution;
  Eigen::MatrixXf rasterized_cost;
  rasterized_cost.resize(dim, dim);
  std::vector<float> log_likelihood_list;
  for(int row_idx = 0; row_idx < dim; row_idx++){
    for(int col_idx = 0; col_idx < dim; col_idx++){
      Vector2f grid_pt(minXorY + col_idx * FLAGS_map_resolution, 
                       minXorY + row_idx * FLAGS_map_resolution);
      float pdf_value = 0.0;
      for(int ranges_idx = 0; ranges_idx < int(ranges.size()); ranges_idx++){
        Vector2f lidar_pt = point_cloud_[ranges_idx];
        pdf_value += statistics::ProbabilityDensityGaussian((float)(grid_pt - lidar_pt).norm(), (float)0.0, (float)FLAGS_sensor_std);
      }
      pdf_value = pdf_value/ranges.size();
      log_pdf_value = std::log(pdf_value);
      rasterized_cost(row_idx, col_idx) = log_pdf_value;
    }
  }
  return(rasterized_cost);
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.

  if (robot_locs_.size() == 0) {
    std::vector<Vector2f> point_cloud_ = GetPointCloud(ranges, range_min, range_max, angle_min, angle_max);
    Eigen::MatrixXf rasterized_cost = GetRasterizedCost(point_cloud_, ranges, range_min, range_max, angle_min, angle_max);
    rasterized_costs.push_back(rasterized_cost);
    robot_locs_.push_back(curr_robot_loc_);
    robot_angles_.push_back(curr_robot_angle_);
    point_clouds.push_back(point_cloud_);
  }
  
  if ((curr_robot_loc_ - robot_locs_.back()).norm() > FLAGS_succ_trans_dist || fabs(curr_robot_angle_ - robot_angles_.back()) > FLAGS_succ_ang_dist) {
    // std::cout << (curr_robot_loc_ - robot_locs_.back()).norm() << endl;
    // std::cout << FLAGS_succ_trans_dist << endl;
    // std::cout << fabs(curr_robot_angle_ - robot_angles_.back()) << endl;
    // std::cout << FLAGS_succ_ang_dist << endl;
    std::vector<Vector2f> point_cloud_ = GetPointCloud(ranges, range_min, range_max, angle_min, angle_max);
    Eigen::MatrixXf rasterized_cost = GetRasterizedCost(point_cloud_, ranges, range_min, range_max, angle_min, angle_max);
    
    Eigen::Vector2f curr_robot_loc_new_;
    float curr_robot_angle_new_;
    std::tie(curr_robot_loc_new_, curr_robot_angle_new_) = GetMostLikelyPose(curr_robot_loc_, curr_robot_angle_, robot_locs_.back(),  robot_angles_.back(),
                                                                     rasterized_costs.back(), point_cloud_, range_max);
    
    std::cout << "Previous pose: (" << robot_locs_.back().x() << ", " << robot_locs_.back().y() << ", "
            << math_util::RadToDeg(robot_angles_.back()) << ")" << endl;
    std::cout << "Predicted pose: (" << curr_robot_loc_.x() << ", " << curr_robot_loc_.y() << ", "
            << math_util::RadToDeg(curr_robot_angle_) << ")" << endl;
    std::cout << "Updated pose: (" << curr_robot_loc_new_.x() << ", " << curr_robot_loc_new_.y() << ", "
            << math_util::RadToDeg(curr_robot_angle_new_) << ")" << endl;
    std::cout << "" << endl;
    
    curr_robot_loc_ = curr_robot_loc_new_;
    curr_robot_angle_ = curr_robot_angle_new_;
    rasterized_costs.push_back(rasterized_cost);
    robot_locs_.push_back(curr_robot_loc_);
    robot_angles_.push_back(curr_robot_angle_);
    point_clouds.push_back(point_cloud_);
  }
}

std::tuple<Eigen::Vector2f, float> SLAM::GetMostLikelyPose(const Eigen::Vector2f& currSLAMPoseLoc,                                                           
                                                           const float& currSLAMPoseAngle,
                                                           const Eigen::Vector2f& prevSLAMPoseLoc,
                                                           const float& prevSLAMPoseAngle,
                                                           Eigen::MatrixXf& rasterized_cost,
                                                           const std::vector<Eigen::Vector2f>& point_cloud_,
                                                           const float& range_max){
  // Defining the likelihood cube. The size of the cube depends on the magnitude
  // of relative motion measured or more precisely the difference in the odometry
  // readings between successive poses.
  // The plausible relative transforms are assumed to be within 95%/ 2 sigma of the 
  // gaussin centered at the relative odometry reading.
  
  Eigen::Rotation2Df prevSLAMPoseRInv(-prevSLAMPoseAngle);
  Eigen::Vector2f relSLAMPoseLoc = prevSLAMPoseRInv*(currSLAMPoseLoc - prevSLAMPoseLoc);
  
  float relSLAMPoseAngle = currSLAMPoseAngle - prevSLAMPoseAngle;
  relSLAMPoseAngle = (fabs(fabs(relSLAMPoseAngle) - 2*M_PI) < fabs(relSLAMPoseAngle)) ? 
                 signbit(relSLAMPoseAngle)*(fabs(relSLAMPoseAngle) -2*M_PI) : relSLAMPoseAngle;

  float sigmaTrans = FLAGS_std_k1*relSLAMPoseLoc.norm() + FLAGS_std_k2*fabs(relSLAMPoseAngle);
  sigmaTrans = std::max(sigmaTrans, (float)0.001);
  float sigmaRot = FLAGS_std_k3*relSLAMPoseLoc.norm() + FLAGS_std_k4*fabs(relSLAMPoseAngle);
  sigmaRot = std::max(sigmaRot, (float)0.001);

  float numOfSigmas = 2;
  int gridSizeX = 2*std::floor(numOfSigmas*sigmaTrans/FLAGS_gridDelX) + 1;
  gridSizeX = 21;
  int gridSizeY = 2*std::floor(numOfSigmas*sigmaTrans/FLAGS_gridDelY) + 1;
  gridSizeY = 21;
  int gridSizeQ = 2*std::floor(numOfSigmas*sigmaRot/FLAGS_gridDelQ) + 1;
  gridSizeQ = 41;

  Eigen::Rotation2Df prevSLAMPoseR(prevSLAMPoseAngle);

  Eigen::Rotation2Df relSLAMPoseR(relSLAMPoseAngle);

  Eigen::Vector2f gridPMin = relSLAMPoseLoc - relSLAMPoseR*Eigen::Vector2f(((gridSizeX - 1)/2)*FLAGS_gridDelX,
                                                                ((gridSizeY - 1)/2)*FLAGS_gridDelY);
  float gridXMin = gridPMin.x();
  float gridYMin = gridPMin.y();
  float gridQMin = relSLAMPoseAngle - ((gridSizeQ - 1)/2)*FLAGS_gridDelQ;

  std::vector<Eigen::MatrixXf> logLikelihoodCube;
  Eigen::MatrixXf logLikelihoodSquare;
  logLikelihoodSquare.resize(gridSizeX, gridSizeY);
  float logLikelihoodMotionModel;
  float logLikelihoodObservationModel;
  float maxLogLikelihood = -INFINITY;
  float likelihoodMotionModelQ;
  std::vector<float> likelihoodMotionModelX;
  std::vector<float> likelihoodMotionModelY;
  Eigen::Vector2f relLoc = relSLAMPoseLoc;
  float relAngle = relSLAMPoseAngle;
  float corrLLOM = -INFINITY;
  float corrLLMM = -INFINITY;

  for(int k = 0; k < gridSizeQ; k++) {
    float CandidateQ = (float)(gridQMin + k*FLAGS_gridDelQ);
    likelihoodMotionModelQ = 
      statistics::ProbabilityDensityGaussian(CandidateQ, relSLAMPoseAngle, sigmaRot);
    
    Eigen::Rotation2Df CandidateR(CandidateQ);
    
    for(int i = 0; i < gridSizeX; i++) {
      float CandidateX;
      Eigen::Vector2f CandidateP;
      for(int j = 0; j < gridSizeY; j++) {
        CandidateP = gridPMin + relSLAMPoseR*Eigen::Vector2f((float)i*FLAGS_gridDelX,
                                                                                  (float)j*FLAGS_gridDelY);
        CandidateX = CandidateP.x();
        float CandidateY = CandidateP.y();
        if(k == 0) {
          likelihoodMotionModelX.push_back(statistics::ProbabilityDensityGaussian(CandidateX, relSLAMPoseLoc.x(), sigmaTrans));
        }
        if(k == 0 && i == 0) {
          likelihoodMotionModelY.push_back(statistics::ProbabilityDensityGaussian(CandidateY, relSLAMPoseLoc.y(), sigmaTrans));
        }

        std::vector<Eigen::Vector2f> Candidate_point_cloud_;
        for(size_t cpc_idx = 0; cpc_idx < point_cloud_.size(); cpc_idx++){
          Vector2f pc_pt = point_cloud_[cpc_idx];
          Vector2f candidate_pc_pt = CandidateR * pc_pt + Vector2f(CandidateX, CandidateY);
          Candidate_point_cloud_.push_back(candidate_pc_pt);
        }

        logLikelihoodObservationModel = GetObservationLikelihood(rasterized_cost, Candidate_point_cloud_, range_max);
        logLikelihoodMotionModel = std::log(likelihoodMotionModelQ*likelihoodMotionModelX[i]*likelihoodMotionModelY[j]);
        logLikelihoodSquare(i, j) = logLikelihoodMotionModel + logLikelihoodObservationModel;
        // logLikelihoodSquare(i, j) = logLikelihoodMotionModel;

        if(k == 0 && i == 0 && j == 0) {
          maxLogLikelihood = logLikelihoodSquare(i, j);
          corrLLOM = logLikelihoodObservationModel;
          corrLLMM = logLikelihoodMotionModel;
        }
        else {
          if (logLikelihoodSquare(i, j) > maxLogLikelihood) {
            maxLogLikelihood = logLikelihoodSquare(i, j);
            corrLLOM = logLikelihoodObservationModel;
            corrLLMM = logLikelihoodMotionModel;
            relLoc = CandidateP;
            relAngle = CandidateQ;
          }
        }
      }
    }
  }
  std::cout << endl;
  std::cout << "MaxLogLikelihood of " << maxLogLikelihood << " observed in Grid of size " << 
    "(" << gridSizeX << ", " << gridSizeY << ", " << gridSizeQ << ")" << endl;
  std::cout << "GridX: (" << gridXMin << ", " <<  gridXMin + gridSizeX*FLAGS_gridDelX << ", " << FLAGS_gridDelX << "), " <<
    "GridY: (" << gridYMin << ", " <<  gridYMin + gridSizeY*FLAGS_gridDelY << ", " << FLAGS_gridDelY << "), " <<
    "GridQ: (" << gridQMin << ", " <<  gridQMin + gridSizeQ*FLAGS_gridDelQ << ", " << FLAGS_gridDelQ << ") " << endl;
  std::cout << "Motion Model MaxLogLikelihood: " << corrLLMM << ", Observation Model MaxLogLikelihood: " << corrLLOM << endl;
  return std::make_tuple(prevSLAMPoseLoc + prevSLAMPoseR*relLoc, prevSLAMPoseAngle + relAngle);
}


std::tuple<Eigen::Vector2f, float> SLAM::DeterministicMotionModel(const Eigen::Vector2f& prevLoc,
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
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.

  // Updating the current robot pose using odometry.
  std::tie(curr_robot_loc_, curr_robot_angle_) = DeterministicMotionModel(curr_robot_loc_, curr_robot_angle_,
    odom_loc, odom_angle, prev_odom_loc_, prev_odom_angle_);

  // std::cout << "ObserveOdometry Pose: (" << curr_robot_loc_.x() << ", " << curr_robot_loc_.y() << ")" << endl;

  // updating the previous odometry values
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

Vector2f SLAM::TransformAndEstimatePointCloud(float x, float y, float theta, Vector2f pt){
  Eigen::Matrix3f T;
  T << cos(theta), -sin(theta), x, 
       sin(theta), cos(theta), y, 
       0, 0, 1;
  Eigen::Vector3f pt3(pt[0], pt[1], 1);
  pt3 = T*pt3;
  return Vector2f(pt3[0], pt3[1]);
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  for(size_t pose_idx = 0; pose_idx < robot_locs_.size(); pose_idx++){
    Vector2f robot_loc = robot_locs_[pose_idx];
    float robot_angle = robot_angles_[pose_idx];
    vector<Vector2f> point_cloud = point_clouds[pose_idx];
    for(size_t pc_idx = 0; pc_idx < point_cloud.size(); pc_idx++){
      Vector2f pc_pt = point_cloud[pc_idx];
      if (pc_pt.norm() > FLAGS_map_range)
        continue;
      Vector2f map_point = TransformAndEstimatePointCloud(robot_loc[0], robot_loc[1], robot_angle, pc_pt);
      map.push_back(map_point);
    }
  }
  return map;
}

}  // namespace slam
