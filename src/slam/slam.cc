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
#include "shared/math/statistics.h"
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

DEFINE_double(gridDelX, 0.005, "5 cm");
DEFINE_double(gridDelY, 0.005, "5 cm");
DEFINE_double(gridDelQ, math_util::DegToRad(5.0), "5 deg");
DEFINE_double(std_k1, 0.2, "Translation dependence on translation standard deviation");
DEFINE_double(std_k2, 0.2, "Rotation dependence on translation standard deviation");
DEFINE_double(std_k3, 0.2, "Translation dependence on rotation standard deviation");
DEFINE_double(std_k4, 0.7, "Rotation dependence on rotation standard deviation");

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

void SLAM:PopulateLikelihoodCube(const Eigen::Vector2f& currSLAMPoseOdomLoc,
                                 const float& currSLAMPoseOdomAngle,
                                 const Eigen::Vector2f& prevSLAMPoseOdomLoc,
                                 const float& prevSLAMPoseOdomAngle) const {
  // Defining the likelihood cube. The size of the cube depends on the magnitude
  // of relative motion measured or more precisely the difference in the odometry
  // readings between successive poses.
  // The plausible relative transforms are assumed to be within 95%/ 2 sigma of the 
  // gaussin centered at the relative odometry reading.
  
  Eigen::Vector2f relOdomLoc = currSLAMPoseOdomLoc - prevSLAMPoseOdomLoc;
  float relOdomAngle = currSLAMPoseOdomAngle - prevSLAMPoseOdomAngle;
  relOdomAngle = (fabs(fabs(relOdomAngle) - 2*M_PI) < fabs(relOdomAngle)) ? 
                 signbit(relOdomAngle)*(fabs(relOdomAngle) -2*M_PI) : relOdomAngle;

  float sigmaTrans = FLAGS_std_k1*relOdomLoc.norm() + FLAGS_std_k2*fabs(relOdomAngle);
  float sigmaRot = FLAGS_std_k3*relOdomLoc.norm() + FLAGS_std_k4*fabs(relOdomAngle);

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


  float gridXMin = relOdomLoc.x() - ((gridSizeX - 1)/2)*FLAGS_gridDelX;
  float gridYMin = relOdomLoc.y() - ((gridSizeY - 1)/2)*FLAGS_gridDelY;
  float gridQMin = relOdomAngle - ((gridSizeX - 1)/2)*FLAGS_gridDelQ;

  std::vector<Eigen::Matrix<float, gridSizeX, gridSizeY>> logLikelihoodCube;
  Eigen::Matrix<float, gridSizeX, gridSizeY> logLikelihoodSquare;
  float logLikelihoodMotionModel;
  float logLikelihoodObservationModel;
  float maxLogLikelihood;
  std::vector<float> likelihoodMotionModelX;
  std::vector<float> likelihoodMotionModelY;
  float likelihoodMotionModelQ;
  float max
  for(int k = 0; k < gridSizeQ; k++) {
    likelihoodMotionModelQ = 
      statistics::ProbabilityDensityGaussian(gridQMin + k*FLAGS_gridDelX, relOdomAngle, sigmaRot);

    for(int i = 0; i < gridSizeX; i++) {
      if(k == 0) {
        likelihoodMotionModelX.push_back(
          statistics::ProbabilityDensityGaussian(gridXMin + i*FLAGS_gridDelX, relOdomLoc.x(), sigmaTrans)
        );
      }
      for(int j = 0; j < gridSizeY; j++) {
        if(k == 0 && i == 0) {
          likelihoodMotionModelY.push_back( 
            statistics::ProbabilityDensityGaussian(gridYMin + j*FLAGS_gridDelY, relOdomLoc.y(), sigmaTrans)
          );
        }
        logLikelihoodMotionModel = std::log(likelihoodMotionModelQ*likelihoodMotionModelX[i]*likelihoodMotionModelX[j]);
        logLikelihoodSquare[i, j] = logLikelihoodMotionModel;//+ logLikelihoodObservationModel;
        if(k == 0 && i == 0 && j == 0)
          maxLogLikelihood = logLikelihoodSquare[i, j];
        else 
          maxLogLikelihood = std::max(maxLogLikelihood, logLikelihoodSquare[i, j]);
      } 
    }
  logLikelihoodCube.push_back(logLikelihoodSquare);
  }
}

std::tuple<Eigen::Vector2f, float> SLAM::DifferentialMotionModel(const Eigen::Vector2f& prevLoc,
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
