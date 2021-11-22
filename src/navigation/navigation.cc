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
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "simple_queue.h"
#include "vector_map/vector_map.h"

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
const float dtgWeight = -0.5;
const float clWeight = 50.0;//200;
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
    nav_goal_set_(false),
    nav_goal_loc_(5, 0),
    nav_goal_angle_(0),
    center_of_curve(0, 0),
    nav_grid_resolution_(0.5),
    local_planner_circle_radius_(1.0),
    hash_coefficient_(10000){
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
   map_ = vector_map::VectorMap(map_file);
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
std::tuple<float, float, float> Navigation::GetPathScoringParams(float curvature_of_turning,
                                                                 const Eigen::Vector2f& localGoal,
                                                                 Eigen::Vector2f& collision_point) {
  float freePathLength = 5.0;
  float distanceToGoal = 0.0;
  float clearance = 5.0;

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
    // front right corner of robot margin boundary
    float radius_of_turning_max = sqrt(x_eval*x_eval + y_eval*y_eval);
    float radius_of_turning_min = radius_of_turning_nominal -
                                  (0.5*(width - track_width) +
                                  0.5*track_width + safety_margin); // side left point of robot margin boundary
    float X = 0.5*wheel_base + 0.5*length + safety_margin;
    float Y = 0.5*width + safety_margin;

    float smallest_angular_distance = 17*M_PI/18; // TODO : Something better to avoid full circular motions
    //float alpha_min = 17*M_PI/18;

    // To compensate for the latency
    // To compensate for the sensing latency
    float rcs_theta_future = (vel_profile[0] * del_t) / radius_of_turning_nominal;
    float rcs_x_future = radius_of_turning_nominal * sin(rcs_theta_future);
    float rcs_y_future = radius_of_turning_nominal * (1 - cos(rcs_theta_future));

    //float rcs_theta_future = 0.0;
    //float rcs_x_future = 0.0;
    //float rcs_y_future = 0.0;

    // Finding the free path length.
    for (unsigned int i = 0; i < point_cloud_.size(); i++) {
      Eigen::Vector2f point_candidate = TransformAndEstimatePointCloud(rcs_x_future,
      rcs_y_future, rcs_theta_future, point_cloud_[i]);

      // Make turning right have same equations as turning left
      point_candidate.y() = direction_of_turning*point_candidate.y();
      float radius_of_point_candidate = (point_candidate - center_of_turning).norm();

      // Check if there exists a possibility of collision with the robot
      if (radius_of_point_candidate <= radius_of_turning_max &&
          radius_of_point_candidate >= radius_of_turning_min) {
        float beta_1 = kInf;
        float beta_2 = kInf;

        if (fabs(X) <= radius_of_point_candidate) {
          beta_1 = asin(X/radius_of_point_candidate); // Returns a value between 0 and pi/2
        }
        if (fabs(radius_of_turning_nominal - Y) <= radius_of_point_candidate)
          beta_2 = acos((radius_of_turning_nominal - Y)/radius_of_point_candidate);
          // Returns a value between 0 and pi/2

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
    for (unsigned int i = 0; i < point_cloud_.size(); i++) {
      // TODO : Alternate to iterating through the point cloud again
      Eigen::Vector2f point_candidate = TransformAndEstimatePointCloud(rcs_x_future,
      rcs_y_future, rcs_theta_future, point_cloud_[i]);

      // Make turning right have same equations as turning left
      point_candidate.y() = direction_of_turning*point_candidate.y();
      float radius_of_point_candidate = (point_candidate - center_of_turning).norm();

      float point_candidate_angular_distance = atan2(point_candidate.x(),
                                                     radius_of_turning_nominal - point_candidate.y());
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
    distanceToGoal = fabs((Eigen::Vector2f(localGoal.x(),localGoal.y()*direction_of_turning)
                           - Vector2f(0, radius_of_turning_nominal)).norm()
                          - radius_of_turning_nominal);

  }

  // Case-2: Moving in a straight line
  else {
    freePathLength = 20.0;
    clearance = 10.0;

    float sweptBound = width / 2 + safety_margin;

    float rcs_theta_future = 0;
    float rcs_x_future = vel_profile[0] * del_t;
    float rcs_y_future = 0;

    for (unsigned int i = 0; i < point_cloud_.size(); i++) {
      Eigen::Vector2f point_candidate = TransformAndEstimatePointCloud(rcs_x_future,
      rcs_y_future, rcs_theta_future, point_cloud_[i]);

      if (point_candidate.x() > 0){
        if (fabs(point_candidate.y()) < sweptBound){
        float pathLength = point_candidate.x();
        freePathLength = std::min(pathLength, freePathLength);
        }
        else{
        clearance = std::min(clearance, fabs(point_candidate.y()) - sweptBound);
        }
      }
    }
    // TODO: Write the function to calculate distance of point from line.
    distanceToGoal = 0.0;
    // distanceToGoal = geometry::DistanceFromLineSegment(localGoal,
    //                                                    Eigen::Vector2f(0.0, 0.0),
    //                                                    Eigen::Vector2f(freePathLength, 0.0));
  }

  return std::make_tuple(freePathLength, distanceToGoal, clearance);
}

float Navigation::OneDTimeOptimalControl(float v0, float distance_remaining){
  float v1;
  if (distance_remaining < 0.0) {
    v1 = 0.0;
    return v1;
  }
  else {
    // Acceleration phase
    v1 = v0 + max_acc*del_t;
    float del_s1 = v1*del_t + 0.5*v1*v1/max_dec;
    float del_s2 = 0.5*v0*v0/max_dec;
    if (v1 <= max_vel && del_s1 < distance_remaining) {
      return v1;
    }
    else if (v0 <= max_vel && del_s2 < distance_remaining) {
      // Cruise phase
      return v0;
    }
    else {
      // Deceleration phase
      v1 = v0 - max_dec*del_t;
      v1 = std::max(v1, (float )0);
      return v1;
    }
  }
}

void Navigation::UpdateVelocityProfile(float last_vel){
  for(int vel_idx = 0; vel_idx < system_lat-1; vel_idx++){
    vel_profile[vel_idx] = vel_profile[vel_idx+1];
  }
  vel_profile[system_lat-1] = last_vel;
}


void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  // Can initialize global planner once start pose of the robot is known.
  std::vector<Eigen::Vector2f> plan;
  nav_goal_loc_ = loc;
  if(localization_initialized_ == true) {
    nav_goal_set_ = true;
    Plan(robot_loc_, loc, plan);
  }
  global_path_ = plan;
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

std::vector<std::pair<int, int>> Navigation::UnobstructedNeighbors(std::pair<int,int> disc_coord) {
  std::vector<std::pair<int, int>> unobstructed_neighbors;
  bool valid_neighbor;
  for (int col_offset = -1; col_offset <= 1; ++col_offset) {
    for (int row_offset = -1; row_offset <= 1; ++row_offset) {
      valid_neighbor = true;
      int row = row_offset + disc_coord.first;
      int col = col_offset + disc_coord.second;

      if ( !((col_offset == 0) && (row_offset == 0))) {
        std::pair<int, int> neighbor{row, col};
        geometry::line2f linep1{DiscCoordToMap(disc_coord), DiscCoordToMap(neighbor)};

        // Is the neighbor unobstructed?
        for (size_t i = 0; i < map_.lines.size(); ++i) {
          if(map_.lines[i].Intersects(linep1)) {
            valid_neighbor = false;
            break;
          }
        }
        if (valid_neighbor)
          unobstructed_neighbors.push_back(neighbor);
      }
    }
  }
  return unobstructed_neighbors;
}


Eigen::Vector2f Navigation::DiscCoordToMap(std::pair<int, int> disc_coord)
{
  Eigen::Vector2f map_coords{nav_grid_resolution_*disc_coord.first, nav_grid_resolution_*disc_coord.second};
  return map_coords;
}


std::pair<int, int> Navigation::DiscretizeCoord( Eigen::Vector2f coord ) {
  return std::make_pair(static_cast<int>(coord.x()/nav_grid_resolution_),
                        static_cast<int>(coord.y()/nav_grid_resolution_));
}

int Navigation::Hash(std::pair<int, int> disc_coord)
{
  return static_cast<int>(disc_coord.first*hash_coefficient_ + disc_coord.second);
}

std::pair<int, int> Navigation::Dehash(int hash) {
  if (hash%hash_coefficient_ > hash_coefficient_/2)
    return std::make_pair(hash/hash_coefficient_ + 1, hash%hash_coefficient_ - hash_coefficient_);
  else if (hash%hash_coefficient_ < -hash_coefficient_/2)
    return std::make_pair(hash/hash_coefficient_ - 1, hash%hash_coefficient_ + hash_coefficient_);
  else
    return std::make_pair(hash/hash_coefficient_, hash%hash_coefficient_);
}

void Navigation::Plan(const Eigen::Vector2f& start_loc,
                      const Eigen::Vector2f& finish_loc,
                      std::vector<Eigen::Vector2f>& plan) {

  // Initialize priority queue
  SimpleQueue<int, float> plan_queue;
  std::pair<int, int> grid_start  = DiscretizeCoord(start_loc);
  plan_queue.Push(Hash(grid_start), 0);
  // std::cout << "Discretized start location: (" << grid_start.first << ", "
  //           << grid_start.second << "); Hash: " << Hash(grid_start) << "\n";

  // Initialize dictionary
  std::map<int, int>backtrack{ {Hash(grid_start), Hash(grid_start)} };
  std::map< int, float >cost_table{ {Hash(grid_start), 0.0} };

  std::pair<int, int> grid_finish = DiscretizeCoord(finish_loc);
  // std::cout << "Discretized finish location: (" << grid_finish.first
  //           << ", " << grid_finish.second << "); Hash: " << Hash(grid_finish) << "\n";

  while(!plan_queue.Empty())
  {
    std::pair<int, int> current_loc = Dehash(plan_queue.Pop());
    // std::cout << "----- Discretized current location: (" << current_loc.first
    //           << ", " << current_loc.second << "); Hash: " << Hash(current_loc) << "\n";

    if(current_loc.first == grid_finish.first && current_loc.second == grid_finish.second) {
      break;
    }

    std::vector<std::pair<int, int>> unobstructed_neighbors = UnobstructedNeighbors(current_loc);

    for(const auto& unobstructed_neighbor: unobstructed_neighbors ) {

      // std::cout << "Discretized neighbour location: (" << unobstructed_neighbor.first << ", "
      //           << unobstructed_neighbor.second << "); Hash: " << Hash(unobstructed_neighbor) << "\n";
      float const new_cost = cost_table.at(Hash(current_loc)) +
                             (DiscCoordToMap(current_loc) -
                             DiscCoordToMap(unobstructed_neighbor)).norm();
      // std::cout << "Cost: " << new_cost << "\n";

      if(cost_table.find(Hash(unobstructed_neighbor)) == cost_table.end() ||
          new_cost < cost_table.at(Hash(unobstructed_neighbor)))
      {
        cost_table[Hash(unobstructed_neighbor)] = new_cost;

        float prio = -(new_cost + (DiscCoordToMap(grid_finish) - DiscCoordToMap(unobstructed_neighbor)).norm());

        // std::cout << "Priority: " << prio << "\n";
        plan_queue.Push(Hash(unobstructed_neighbor), prio);
        backtrack[Hash(unobstructed_neighbor)] = Hash(current_loc);
      }
    }
  }

  // Create path
  int current = Hash(grid_finish);
  plan.push_back(finish_loc);
  while(current != Hash(grid_start) )
  {
    plan.push_back(DiscCoordToMap(Dehash(current)));
    current = backtrack.at(current);
  }
  plan.push_back(DiscCoordToMap(Dehash(current)));
  plan.push_back(start_loc);
  reverse(plan.begin(), plan.end());
  return;
}



Vector2f Navigation::TransformAndEstimatePointCloud(float x, float y, float theta, Vector2f pt) {
  Eigen::Affine2f T;
  T = Eigen::Translation2f(x, y)*Eigen::Rotation2Df(theta);
  return T.inverse()*pt;
}

bool Navigation::FindPathCircleIntersection(const std::vector<Eigen::Vector2f>& path,
                                        const Eigen::Vector2f& circle_center,
                                        float radius,
                                        Eigen::Vector2f& point) {
  for(uint32_t i = 0; i < path.size(); i++) {
    if(i >= 1) {
      bool intersects = FindLineSegmentCircleIntersections(path[i-1], path[i], circle_center, radius,
                                                           point);
      if (intersects)
        return true;
    }
  }
  return false;
}

bool Navigation::FindLineSegmentCircleIntersections(const Eigen::Vector2f& point1,
                                             const Eigen::Vector2f& point2,
                                             const Eigen::Vector2f& center,
                                             float radius,
                                             Eigen::Vector2f& intersection) {
  float dx, dy, A, B, C, det, t;

  dx = point2.x() - point1.x();
  dy = point2.y() - point1.y();

  A = dx*dx + dy*dy;
  B = 2.0*(dx*(point1.x() - center.x()) + dy*(point1.y() - center.y()));
  C = (point1.x() - center.x())*(point1.x() - center.x()) +
      (point1.y() - center.y())*(point1.y() - center.y()) -
      radius*radius;

  det = B*B - 4.0*A*C;
  if ((A <= kEpsilon) || (det < 0)) {
    // No real solutions.
    return false;
  }
  else if (fabs(det) < kEpsilon) {
    // One solution.
    t = -B/(2.0*A);
    intersection << point1.x() + t*dx, point1.y() + t*dy;
    if (geometry::IsBetween(point1, point2, intersection, kEpsilon))
      return true;
  }
  else {
    // Two solutions.
    t = ((-B + std::sqrt(det))/(2.0*A));
    Eigen::Vector2f intersection1 = {point1.x() + t*dx, point1.y() + t*dy};
    t = ((-B - std::sqrt(det))/(2.0*A));
    Eigen::Vector2f intersection2 = {point1.x() + t*dx, point1.y() + t*dy};

    if (geometry::IsBetween(point1, point2, intersection1, kEpsilon)) {
      if (geometry::IsBetween(point1, point2, intersection2, kEpsilon)) {
        if ((intersection1 - center).norm() <= (intersection2 - center).norm()) {
          intersection = intersection1;
          return true;
        }
        else {
          intersection = intersection2;
          return true;
        }
      }
      else {
        intersection = intersection1;
        return true;
      }
    }
    else if (geometry::IsBetween(point1, point2, intersection2, kEpsilon)) {
      intersection = intersection2;
      return true;
    }
    else
      return false;
  }
  return false;
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // Get the location of the robot with respect to its initial reference frame.
  Eigen::Vector2f relPos;
  float relAngle;
  std::tie(relPos, relAngle) = getRelativePose(odom_start_loc_, odom_start_angle_, odom_loc_, odom_angle_);

  if (nav_goal_set_) {
    // Global planner.
    std::vector<Eigen::Vector2f> plan;
    Plan(robot_loc_, nav_goal_loc_, plan);
    global_path_ = plan;

    // Integrating global planner with local planner.
    Eigen::Vector2f local_goal = {5.0, 0.0};
    nav_complete_ = !FindPathCircleIntersection(global_path_,
                                            robot_loc_,
                                            local_planner_circle_radius_,
                                            local_goal);
    // visualization::DrawCross(local_goal, 0.1, 0x000000, global_viz_msg_);
    visualization::DrawArc(robot_loc_, local_planner_circle_radius_, 0.0, M_2PI, 0x000000, global_viz_msg_);

    local_goal = TransformAndEstimatePointCloud(robot_loc_.x(),
                                                robot_loc_.y(),
                                                robot_angle_,
                                                local_goal);
    // visualization::DrawCross(Eigen::Vector2f(1.5, 0.0), 0.2, 0x0000FF, local_viz_msg_);
    visualization::DrawCross(local_goal, 0.1, 0xFF0000, local_viz_msg_);
    // std::cout << "Local Goal: "<< local_goal.x() << ", "
    //           << local_goal.y() << ")" << "\n";

    float distance_remaining = 0.0;
    if (nav_complete_) {
      distance_remaining = 0.0;
      // TODO : Maintain previous curvature
      drive_msg_.curvature = 0.0;
    }
    else {
      // Local planner with obstacle avoidance.
      Eigen::Vector2f collision_point; // to visualize the first point of collision with an obstacle given a path
      Eigen::Vector2f collision_point_candidate;
      distance_remaining = 0.0; // setting a minimum value of zero
      float best_score = -kInf;

      float curvature_candidate;
      float freePathLengthCandidate;
      float distanceToGoalCandidate;
      float clearanceCandidate;

      for (curvature_candidate = 1.05; curvature_candidate >= -1.06;
           curvature_candidate = curvature_candidate - 0.1) {
        std::tie(freePathLengthCandidate, distanceToGoalCandidate, clearanceCandidate) =
          GetPathScoringParams(curvature_candidate,
                               local_goal,
                               collision_point_candidate);
        float score = freePathLengthCandidate +
                      dtgWeight*distanceToGoalCandidate +
                      clWeight*clearanceCandidate;

        // std::cout << "C: "<< curvature_candidate << ", FPL: " << freePathLengthCandidate
        //           << ", DTG: " << dtgWeight*distanceToGoalCandidate
        //           << ", Cl: " << clWeight*clearanceCandidate << ", S: " <<score << "\n";
        // Choosing the arc/line with the best score
        if (score > best_score) {
          best_score = score;
          distance_remaining = freePathLengthCandidate;
          collision_point = collision_point_candidate;
          drive_msg_.curvature = curvature_candidate; // choosing curvature with best score
        }
      }
      visualization::DrawPath(drive_msg_.curvature, freePathLengthCandidate, 0xFFA500, local_viz_msg_);
    }

    // // Adjust the distance traveled by taking into account the latency compensation.
    // vel_sum = 0;
    // for(int vel_idx = 0; vel_idx < system_lat; vel_idx++)
    //   vel_sum += vel_profile[vel_idx];
    // distance_remaining = distance_remaining - (vel_sum) * del_t;

    // Time optimal control.
    float v0 = vel_profile[system_lat - 1];
    drive_msg_.velocity = OneDTimeOptimalControl(v0, distance_remaining);
    UpdateVelocityProfile(drive_msg_.velocity);
  }
  else {
    drive_msg_.velocity = 0.0;
    drive_msg_.curvature = 0.0;
  }

  // Create visualizations.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  visualization::DrawRobotMargin(length, width, wheel_base, track_width, safety_margin, local_viz_msg_);
    // float radius_of_turning_min = fabs(1/curvature_candidate) - (0.5*(width - track_width) +
    //                               0.5*track_width + safety_margin);
    // visualization::DrawPathOption(curvature_candidate, radius_of_turning_min*freePathLengthCandidate,
    //                               0.5*width+safety_margin, local_viz_msg_);


  for(uint32_t i = 0; i < global_path_.size(); i++) {
    if (i >= 1) {
      visualization::DrawLine(global_path_[i-1], global_path_[i], 0x00FF00, global_viz_msg_);
    }
  }

  std::cout << "Speed: " << drive_msg_.velocity << "\n";
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);

}

}  // namespace navigation
