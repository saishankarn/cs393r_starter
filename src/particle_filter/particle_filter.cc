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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
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
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 2, "Number of particles");
DEFINE_double(std_k1, 0*0.05, "Translation dependence on translationnal motion model standard deviation");
DEFINE_double(std_k2, 0*0.5*M_PI/180.0, "Rotation dependence on translational motion model standard deviation");
DEFINE_double(std_k3, 0*0.05, "Translation dependence on rotational motion model standard deviation");
DEFINE_double(std_k4, 0*0.5*M_PI/180.0, "Rotation dependence on rotational motion model standard deviation");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    prev_map_loc_(0, 0),
    prev_map_angle_(0) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
    const float angle,
    int num_ranges,
    float range_min,
    float range_max,
    float angle_min,
    float angle_max,
    vector<Vector2f>* scan_ptr) {
    vector<Vector2f>& scan = *scan_ptr;
    // Compute what the predicted point cloud would be, if the car was at the pose
    // loc, angle, with the sensor characteristics defined by the provided
    // parameters.
    // This is NOT the motion model predict step: it is the prediction of the
    // expected observations, to be used for the update step.
    float current_angle;
    bool intersects;
    // Note: The returned values must be set using the `scan` variable:
    scan.resize(num_ranges);
    // Fill in the entries of scan using array writes, e.g. scan[i] = ...
    for (size_t i = 0; i < scan.size(); ++i) {
        scan[i] = Vector2f(0, 0);
        current_angle = angle_min + (i * (angle_max - angle_min)) / scan.size();
        line2f scan_line(loc[0] + range_min * cos(current_angle), loc[1] + range_min * cos(current_angle),
            loc[0] + range_max * cos(current_angle), loc[1] + range_max * cos(current_angle));
        for (size_t i = 0; i < map_.lines.size(); ++i) {
            const line2f map_line = map_.lines[i];
            intersects = map_line.Intersects(scan_line);
            Vector2f intersection_point; // Return variable
            Vector2f end_of_line(loc[0] + range_max * cos(current_angle), loc[1] + range_max * cos(current_angle));
            intersects = map_line.Intersection(my_line, &intersection_point);
            if (intersects)
            {
                scan[i] = intersection_point;
            }
            else {
                scan[i] = end_of_line
            }
        }
    }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  // float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //        x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
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
  float angle = prevAngle + deltaAngle; // Angle in Map frame.

  // TODO : Implement different scaling of variance for axes x and y.
  // Adding uncertainty to the prediction.
  loc = loc + Eigen::Vector2f(rng_.Gaussian(0, FLAGS_std_k1*deltaLoc.norm() + FLAGS_std_k2*fabs(deltaAngle)), 
                              rng_.Gaussian(0, FLAGS_std_k1*deltaLoc.norm() + FLAGS_std_k2*fabs(deltaAngle)));
  angle = angle + rng_.Gaussian(0, FLAGS_std_k3*deltaLoc.norm() + FLAGS_std_k4*fabs(deltaAngle));

  return std::make_tuple(loc, angle);
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  
  // For the first time instance, initialize the previous odometry values to the
  // current values.
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
  }

  std::for_each(particles_.begin(), particles_.end(), [&](Particle &particle) {
    Eigen::Vector2f loc;
    float angle;
    std::tie(loc, angle) = MotionModel(particle.loc, particle.angle,
    odom_loc, odom_angle, prev_odom_loc_, prev_odom_angle_);
    particle.loc = loc;
    particle.angle = angle;
    std::cout << "Particle loc: (" << particle.loc.x() <<", " << particle.loc.y() << ")"
    << " and angle: " << particles_[0].angle << "\n";
  });
  
  // std::cout << "Particle loc: " << particles_.size() << " and angle: " << particles_.size() << "\n";
  


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  // float x = rng_.Gaussian(0.0, 2.0);
  // printf("Random number drawn from Gaussian distribution with 0 mean and "
        //  "standard deviation of 2 : %f\n", x);
  
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle. The location and 
  // angle are in the Map coordinate frame
  // some distribution around the provided location and angle.
  map_.Load(map_file);
  prev_map_loc_ = loc;
  prev_map_angle_ = angle;

  // Initializing particles around the initial location of the robot.
  for (int i = 0; i < FLAGS_num_particles; i++) {
    Particle particle_generated;

    // Drawing the initial particles from a Gaussian. 95% of the particles are within 10cm and 1deg
    // from the initial pose of the base link frame.
    particle_generated.loc = Eigen::Vector2f(rng_.Gaussian(loc.x(), 0*0.05), rng_.Gaussian(loc.y(), 0*0.05));
    particle_generated.angle = rng_.Gaussian(angle, 0*0.5*M_PI/180.0);
    
    // Initializing the observation likelihood (weights) and the belief
    particle_generated.weight = 1.0;
    particle_generated.belief = (double) 1/FLAGS_num_particles;
    
    particles_.push_back(particle_generated);
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;
}


}  // namespace particle_filter
