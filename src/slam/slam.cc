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

#include "slam.h"

#include "vector_map/vector_map.h"

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

DEFINE_double(map_resolution, 0.1, "Rasterized cost map resolution");
DEFINE_double(sensor_std, 0.15, "standard deviation of sensor");

namespace slam {

DEFINE_double(gridDelX, 0.005, "5 cm");
DEFINE_double(gridDelY, 0.005, "5 cm");
DEFINE_double(gridDelQ, math_util::DegToRad(5.0), "5 deg");
DEFINE_double(succ_trans_dist, 0.5, "50 cm");
DEFINE_double(succ_ang_dist, math_util::DegToRad(45.0), "45 deg");
DEFINE_double(std_k1, 0.2, "Translation dependence on translation standard deviation");
DEFINE_double(std_k2, 0.2, "Rotation dependence on translation standard deviation");
DEFINE_double(std_k3, 0.2, "Translation dependence on rotation standard deviation");
DEFINE_double(std_k4, 0.7, "Rotation dependence on rotation standard deviation");

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
                                    float range_min,
                                    float range_max,
                                    float angle_min,
                                    float angle_max){
  // returns the observation likelihood scores of a given point_cloud_ using the rasterized_cost
  float obs_log_likelihood = 0.0;
  int dim = int(2 * range_max / FLAGS_map_resolution); // 600
  for(int pc_idx = 0; pc_idx < int(point_cloud_.size()); pc_idx++){
    Vector2f pt = point_cloud_[pc_idx];
    Vector2f grid_pt(int((range_max - pt[0]) / FLAGS_map_resolution),
                     int((range_max - pt[1]) / FLAGS_map_resolution));
    if (grid_pt[0] >= 0 && grid_pt[0] < dim){
      if (grid_pt[1] >= 0 && grid_pt[1] < dim){
        obs_log_likelihood += rasterized_cost(grid_pt[0], grid_pt[1]);
      }
    }
  }
  return obs_log_likelihood;
}

Eigen::MatrixXf SLAM::GetRasterizedCost(const vector<float>& ranges,
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
  std::vector<float> log_likelihood_list;
  for(int row_idx = 0; row_idx < dim; row_idx++){
    for(int col_idx = 0; col_idx < dim; col_idx++){
      Vector2f grid_pt(range_max - row_idx * FLAGS_map_resolution, 
                       range_max - col_idx * FLAGS_map_resolution);
      float sum_log_likelihood = 0.0;
      float pdf_value = 0.0;
      for(int ranges_idx = 0; ranges_idx < int(ranges.size()); ranges_idx++){
        Vector2f lidar_pt = point_cloud_[ranges_idx];
        pdf_value += statistics::ProbabilityDensityGaussian((float)(grid_pt - lidar_pt).norm(), (float)0.0, (float)FLAGS_sensor_std);
      }
      pdf_value = pdf_value/ranges.size();
      log_likelihood_list.push_back(std::log(pdf_value));
    }
  }
  Eigen::MatrixXf rasterized_cost = Eigen::MatrixXf::Map(&log_likelihood_list[0], dim, dim);
  rasterized_cost.transposeInPlace();
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

  std::cout << "ObserveLaser Pose: (" << curr_robot_loc_.x() << ", " << curr_robot_loc_.y() << endl;

  if (robot_locs_.size() == 0) {
    Eigen::MatrixXf rasterized_cost = GetRasterizedCost(ranges, rasterized_cost, range_min, range_max, angle_min, angle_max);
    rasterized_costs.push_back(rasterized_cost);
    robot_locs_.push_back(curr_robot_loc_);
    robot_angles_.push_back(curr_robot_angle_);
  }
  
  if ((curr_robot_loc_ - robot_locs_.back()).norm() > FLAGS_succ_trans_dist || fabs(curr_robot_angle_ - robot_angles_.back()) > FLAGS_succ_ang_dist) {
    // std::cout << (curr_robot_loc_ - robot_locs_.back()).norm() << endl;
    // std::cout << FLAGS_succ_trans_dist << endl;
    // std::cout << fabs(curr_robot_angle_ - robot_angles_.back()) << endl;
    // std::cout << FLAGS_succ_ang_dist << endl;
    Eigen::MatrixXf rasterized_cost = GetRasterizedCost(ranges, range_min, range_max, angle_min, angle_max);
    std::tie(curr_robot_loc_, curr_robot_angle_) = GetMostLikelyPose(curr_robot_loc_, curr_robot_angle_, robot_locs_.back(),  robot_angles_.back(),
                                                                     rasterized_cost, point_cloud_, range_max);
    rasterized_costs.push_back(rasterized_cost);
    robot_locs_.push_back(curr_robot_loc_);
    robot_angles_.push_back(curr_robot_angle_);
  }
}

std::tuple<Eigen::Vector2f, float> SLAM::GetMostLikelyPose(const Eigen::Vector2f& currSLAMPoseLoc,                                                           
                                                           const float& currSLAMPoseAngle,
                                                           const Eigen::Vector2f& prevSLAMPoseLoc,
                                                           const float& prevSLAMPoseAngle,
                                                           const Eigen::MatrixXf& rasterized_cost,
                                                           const vector<Vector2f>& point_cloud_,
                                                           const float& range_max) const {
  // Defining the likelihood cube. The size of the cube depends on the magnitude
  // of relative motion measured or more precisely the difference in the odometry
  // readings between successive poses.
  // The plausible relative transforms are assumed to be within 95%/ 2 sigma of the 
  // gaussin centered at the relative odometry reading.
  
  Eigen::Vector2f relSLAMPoseLoc = currSLAMPoseLoc - prevSLAMPoseLoc;
  float relSLAMPoseAngle = currSLAMPoseAngle - prevSLAMPoseAngle;
  relSLAMPoseAngle = (fabs(fabs(relSLAMPoseAngle) - 2*M_PI) < fabs(relSLAMPoseAngle)) ? 
                 signbit(relSLAMPoseAngle)*(fabs(relSLAMPoseAngle) -2*M_PI) : relSLAMPoseAngle;

  float sigmaTrans = FLAGS_std_k1*relSLAMPoseLoc.norm() + FLAGS_std_k2*fabs(relSLAMPoseAngle);
  float sigmaRot = FLAGS_std_k3*relSLAMPoseLoc.norm() + FLAGS_std_k4*fabs(relSLAMPoseAngle);

  float numOfSigmas = 2;
  int gridSizeX;
  int gridSizeY;
  int gridSizeQ;
  std::remquo(numOfSigmas*sigmaTrans, FLAGS_gridDelX, &gridSizeX);
  std::remquo(numOfSigmas*sigmaTrans, FLAGS_gridDelY, &gridSizeY);
  std::remquo(numOfSigmas*sigmaRot, FLAGS_gridDelQ, &gridSizeQ);
  gridSizeX = 2*gridSizeX + 1;
  gridSizeY = 2*gridSizeY + 1;
  gridSizeQ = 2*gridSizeQ + 1;


  float gridXMin = relSLAMPoseLoc.x() - ((gridSizeX - 1)/2)*FLAGS_gridDelX;
  float gridYMin = relSLAMPoseLoc.y() - ((gridSizeY - 1)/2)*FLAGS_gridDelY;
  float gridQMin = relSLAMPoseAngle - ((gridSizeQ - 1)/2)*FLAGS_gridDelQ;

  std::vector<Eigen::MatrixXf> logLikelihoodCube;
  Eigen::MatrixXf logLikelihoodSquare;
  logLikelihoodSquare.resize(gridSizeX, gridSizeY);
  float logLikelihoodMotionModel;
  // float logLikelihoodObservationModel;
  float maxLogLikelihood;
  float likelihoodMotionModelQ;
  std::vector<float> likelihoodMotionModelX;
  std::vector<float> likelihoodMotionModelY;
  float loc = relSLAMPoseLoc;
  float angle = relSLAMPoseAngle;

  for(int k = 0; k < gridSizeQ; k++) {
    likelihoodMotionModelQ = 
      statistics::ProbabilityDensityGaussian((float)(gridQMin + k*FLAGS_gridDelQ), relSLAMPoseAngle, sigmaRot);

    for(int i = 0; i < gridSizeX; i++) {
      if(k == 0) {
        likelihoodMotionModelX.push_back(
          statistics::ProbabilityDensityGaussian((float)(gridXMin + i*FLAGS_gridDelX), relSLAMPoseLoc.x(), sigmaTrans)
        );
      }
      for(int j = 0; j < gridSizeY; j++) {
        if(k == 0 && i == 0) {
          likelihoodMotionModelY.push_back( 
            statistics::ProbabilityDensityGaussian((float)(gridYMin + j*FLAGS_gridDelY), relSLAMPoseLoc.y(), sigmaTrans)
          );
        }
        logLikelihoodMotionModel = std::log(likelihoodMotionModelQ*likelihoodMotionModelX[i]*likelihoodMotionModelY[j]);
        logLikelihoodSquare(i, j) = logLikelihoodMotionModel;//+ logLikelihoodObservationModel;
        if(k == 0 && i == 0 && j == 0)
          maxLogLikelihood = logLikelihoodSquare(i, j);
        else {
          if (logLikelihoodSquare(i, j) > maxLogLikelihood) {
            maxLogLikelihood = logLikelihoodSquare(i, j);
            loc = Eigen::Vector2f((float)(gridXMin + i*FLAGS_gridDelX), (float)(gridYMin + j*FLAGS_gridDelY));
            angle = (float)(gridQMin + k*FLAGS_gridDelQ);
          }
        }
      }
    }
  logLikelihoodCube.push_back(logLikelihoodSquare);
  //std::cout << logLikelihoodCube[k] << endl;
  }
  // std::cout << "Current pose: (" << currSLAMPoseLoc.x() << ", " << currSLAMPoseLoc.y() << ", "
  //           << math_util::RadToDeg(currSLAMPoseAngle) << ")" << endl;
  // std::cout << "Previous pose: (" << prevSLAMPoseLoc.x() << ", " << prevSLAMPoseLoc.y() << ", "
  //           << math_util::RadToDeg(prevSLAMPoseAngle) << ")" << endl;
  // std::cout << "-------------------------------------------------------------------------" << endl;
  return std::make_tuple(loc, angle);
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

  std::cout << "ObserveOdometry Pose: (" << curr_robot_loc_.x() << ", " << curr_robot_loc_.y() << endl;

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
