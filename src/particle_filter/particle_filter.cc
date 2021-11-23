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
#include <math.h>
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

DEFINE_double(num_particles, 50, "Number of particles");
DEFINE_double(std_k1, 0.2, "Translation dependence on translationnal motion model standard deviation");
DEFINE_double(std_k2, 0.2, "Rotation dependence on translational motion model standard deviation");
DEFINE_double(std_k3, 0.2, "Translation dependence on rotational motion model standard deviation");//0.1
DEFINE_double(std_k4, 0.7, "Rotation dependence on rotational motion model standard deviation");//0.1
//DEFINE_double(gamma_sensor, 0.1, "Independence factor");

DEFINE_double(d_long, 1, "D long");//15
DEFINE_double(d_short, 0.01, "D short");//1
DEFINE_double(sensor_std, 0.3, "standard deviation of sensor");//0.3

extern const size_t num_scans = 100;

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    prev_map_loc_(0, 0),
    prev_map_angle_(0),
    noPredictWOUpdate(0) {}

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
    // Note: The returned values must be set using the `scan` variable:
    scan.resize(num_ranges);
    // Fill in the entries of scan using array writes, e.g. scan[i] = ...
    for (size_t i = 0; i < scan.size(); ++i) {
        scan[i] =  GetOnePoint(loc, angle, num_ranges, i, range_min, range_max, angle_min, angle_max);
        //cout << scan[i] << '\n' ;
    }
}



Vector2f ParticleFilter::GetOnePoint(const Vector2f& loc,
                                     const float angle,
                                     int num_ranges,
                                     int range_index,
                                     float range_min,
                                     float range_max,
                                     float angle_min,
                                     float angle_max) {

  bool intersects;
  float line_length, new_line_length, current_angle;
  Vector2f end_of_line;
  Vector2f output_point(0, 0);
  Vector2f lidar_loc(loc[0] + 0.2 * cos(angle), loc[1] + 0.2 * sin(angle));
  current_angle = angle + (angle_min + (range_index * (angle_max - angle_min)) / num_ranges);
  //cout << "lidar_loc : " << lidar_loc[0] << " " << lidar_loc[1] << " " << '\n';
  line2f scan_line(lidar_loc[0] + range_min * cos(current_angle), lidar_loc[1] + range_min * sin(current_angle),
                   lidar_loc[0] + range_max * cos(current_angle), lidar_loc[1] + range_max * sin(current_angle));
  line_length = scan_line.Length();

  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    intersects = map_line.Intersects(scan_line);
    Vector2f intersection_point; // Return variable
    end_of_line(lidar_loc[0] + range_max * cos(current_angle), lidar_loc[1] + range_max * sin(current_angle));
    intersects = map_line.Intersection(scan_line, &intersection_point);
    if (intersects){
      line2f new_line(lidar_loc[0] + range_min * cos(current_angle), lidar_loc[1] + range_min * sin(current_angle),
                      intersection_point.x(), intersection_point.y());
      new_line_length = new_line.Length();
      if (line_length > new_line_length) {
        line_length = new_line_length;
        end_of_line = intersection_point;
      }
    }
  }

  return end_of_line;
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

  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  GetLocation(&robot_loc, &robot_angle);

  Vector2f point_at_i;
  double distance_to_point;
  double total_weight_update;
  double weight_update_at_index;

  // Debug variables
  size_t num_dlong = 0;
  size_t num_dshort = 0;
  size_t num_gauss = 0;

  // weights will always be log-liklihoods as opposed to real probabilities
  total_weight_update = 0.0;
  weight_update_at_index = 0.0;

  for(size_t i = 0; i < ranges.size(); i+=1){
    if(ranges[i] >= range_max || ranges[i] <= range_min){
      continue;
    }
    // cout << "num ranges in update " << ranges.size() << '\n';
    point_at_i = GetOnePoint(p_ptr->loc, p_ptr->angle, ranges.size(), i, range_min, range_max, angle_min, angle_max);
    distance_to_point = (point_at_i - robot_loc).norm();

    //cout << "distance to point " << distance_to_point << '\n';
    //cout << "range " << ranges[i] << '\n';

    if(ranges[i] > distance_to_point + FLAGS_d_long){
      //cout << "here at dlong" << '\n';
      weight_update_at_index = - (pow(FLAGS_d_long, 2) / pow(FLAGS_sensor_std,2));
      num_dlong++;
    }

    else if(ranges[i] < distance_to_point - FLAGS_d_short){
      //cout << "here at dshort" << '\n';
      weight_update_at_index = - (pow(FLAGS_d_short, 2) / pow(FLAGS_sensor_std,2));
      num_dshort++;
    }

    else{
      //cout << "here at normal" << '\n';
      weight_update_at_index = - (pow(ranges[i] - distance_to_point, 2) / pow(FLAGS_sensor_std,2));
      num_gauss++;
    }
    total_weight_update += weight_update_at_index;
  }
  //cout << "total_weight_update : " << total_weight_update <<  "               " << gamma_sensor << '\n';
  //p.weight = p.weight + total_weight_update;
  p_ptr->weight = 0.1 * total_weight_update;
  // std::cout << "dlong: " << num_dlong << ", gauss: " << num_gauss << ", dshort: " << num_dshort << "\n";
}

// void ParticleFilter::PlotObservationLikelihoodContour(float delta, )

void ParticleFilter::Resample() {

  vector<float> bins;
  vector<Particle> new_particles_;

  double sum_weights = 0;
  for (Particle& p : particles_){
    bins.push_back(sum_weights);
    sum_weights = sum_weights + p.weight;
  }
  bins.push_back(sum_weights);

  // Vanilla resampling
  if (particles_.size() != 0){
    for (int new_p_idx = 0; new_p_idx < FLAGS_num_particles; new_p_idx++){
      float x_bin = rng_.UniformRandom(0, 1);
      for (size_t bin_idx = 0; bin_idx < bins.size() - 1; bin_idx++) {
        if (bins[bin_idx] <= x_bin && x_bin <=bins[bin_idx+1]){
          Particle new_particle = particles_[bin_idx];
          new_particle.weight = 1.0;
          new_particles_.push_back(new_particle);
        }
      }
    }
    particles_ = new_particles_;
  }

  // // Low variance resampling
  // float x_bin = rng_.UniformRandom(0, 1);
  // size_t bin_start = findBin(x_bin, &bins);
  // Particle new_particle = particles_[bin_start];
  // size_t bin_id = bin_start;
  // while(bin_id < bin.size()) {

  // }

}

// size_t findBin(float x_bin, const std:vector<float>& bins) const{
//   for (auto it = bins.begin(); it != bins.end(); ++it) {
//     if (x_bin <= 0.0)
//       return 0;
//     else if (x_bin <= *it)
//       return std::distance(bins.begin(), it) - 1;
//     else
//       return bin.size() - 1;
//   }
// }

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  // downsampling the ranges vector
  float mod_angle_min = -60*M_PI/180;
  float mod_angle_max = 60*M_PI/180;

  float angle_increment = (angle_max - angle_min) / ranges.size();
  size_t skip_interval = (mod_angle_max - mod_angle_min)/num_scans/angle_increment;
  // std::cout << "skip interval: " << skip_interval;
  size_t ranges_bias_idx_min = int((mod_angle_min - angle_min) / angle_increment);
  size_t ranges_bias_idx_max = int((mod_angle_max - angle_min) / angle_increment);

  vector<float> downsampled_ranges;
  for (size_t req_ds_idx = ranges_bias_idx_min; req_ds_idx < ranges_bias_idx_max; req_ds_idx = req_ds_idx + skip_interval)
    downsampled_ranges.push_back(ranges[req_ds_idx]);

  // cout << "downsampled ranges size : " << downsampled_ranges.size() << '\n';

  if (noPredictWOUpdate > 1) {
  // updating the weights of the particles
  double sum_weights = 0.0;
  double max_weight = -1000000.0;
  size_t i_debug = 0;
  for (Particle& p : particles_){
    // std::cout << "Particle number: " << i_debug << "\n";
    i_debug++;
    Update(downsampled_ranges, range_min, range_max, mod_angle_min, mod_angle_max, &p);
    if (max_weight < p.weight){
      max_weight = p.weight;
    }
  }

  for (Particle& p : particles_){
    p.weight = p.weight - max_weight;
    p.weight = exp(p.weight);
    sum_weights = sum_weights + p.weight;
  }

  for (Particle& p : particles_){
    p.weight = p.weight / sum_weights;
  }

  Resample();

  noPredictWOUpdate = 0;
  }


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
  });

  noPredictWOUpdate++;

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
    particle_generated.loc = Eigen::Vector2f(rng_.Gaussian(loc.x(), 0.05), rng_.Gaussian(loc.y(), 0.05));
    particle_generated.angle = rng_.Gaussian(angle, 0.5*M_PI/180.0);

    // Initializing the observation likelihood (weights) and the belief
    particle_generated.weight = 1.0;

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

  for (auto it = begin (particles_); it != end (particles_); ++it) {
    loc = loc + it->loc;
    angle = angle + it->angle;
  }
  loc = loc / FLAGS_num_particles;
  angle = angle / FLAGS_num_particles;
}


}  // namespace particle_filter