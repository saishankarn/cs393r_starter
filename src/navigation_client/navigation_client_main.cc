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
\file    navigation_main.cc
\brief   Main entry point for reference Navigation implementation
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h> 
#include <vector>
#include <ctime>
#include <iostream>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"

#include "navigation_client.h"

using amrl_msgs::Localization2DMsg;
using geometry_msgs::PointStamped;
using math_util::DegToRad;
using math_util::RadToDeg;
using navigation_client::Navigation_client;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using std::string;
using std::vector;
using Eigen::Vector2f;
using namespace std;

// Create command line arguments
//DEFINE_string(laser_topic, "scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "odom", "Name of ROS topic for odometry data");
DEFINE_string(loc_topic, "localization", "Name of ROS topic for localization");
DEFINE_string(init_topic,
              "initialpose",
              "Name of ROS topic for initialization");
DEFINE_string(map, "maps/GDC1.txt", "Name of vector map file");
DEFINE_string(server_topic, "server_path_params", "Name of ROS topic for path params from the server side");

// Logging File
static std::ofstream log_file_spin_loop("log/2023-jan-28/spin_loop_" + std::to_string(std::time(0)) + ".csv");

bool run_ = true;
//sensor_msgs::LaserScan last_laser_msg_;
Navigation_client* navigation_client_ = nullptr;

void ServerPathParamCallback(const geometry_msgs::PointStamped& msg) {
  if (FLAGS_v > 0) {
    printf("ServerPathParam corresponding to timet=%f\n", msg.header.stamp.toSec());
  }
  navigation_client::srvMsgStruct srvMsg;
  srvMsg.distance_remaining = msg.point.x;
  srvMsg.curvature = msg.point.y;
  srvMsg.scan_time_stamp = msg.header.stamp;

  navigation_client_->QueueSrvMsg(srvMsg);
  // std::cout << "Laser scan time-stamp" << srvMsg.scan_time_stamp << std::endl;
  std::cout << "Network time delay: " << (ros::Time::now() - srvMsg.scan_time_stamp)*1000.0 << std::endl;
  
  log_file_spin_loop << (ros::Time::now() - srvMsg.scan_time_stamp)*1000.0
                     << '\n';
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  navigation_client_->UpdateOdometry(
      Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y),
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
      Vector2f(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
      msg.twist.twist.angular.z);
}

void GoToCallback(const geometry_msgs::PoseStamped& msg) {
  const Vector2f loc(msg.pose.position.x, msg.pose.position.y);
  const float angle =
      2.0 * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
  printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), angle);
  navigation_client_->SetNavGoal(loc, angle);
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

void LocalizationCallback(const amrl_msgs::Localization2DMsg msg) {
  if (FLAGS_v > 0) {
    printf("Localization t=%f\n", GetWallTime());
  }
  navigation_client_->UpdateLocation(Vector2f(msg.pose.x, msg.pose.y), msg.pose.theta);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler); 
  // Initialize ROS.
  ros::init(argc, argv, "navigation_client", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  navigation_client_ = new Navigation_client(FLAGS_map, &n);

  ros::Subscriber velocity_sub =
      n.subscribe(FLAGS_odom_topic, 1, &OdometryCallback);
  ros::Subscriber localization_sub =
      n.subscribe(FLAGS_loc_topic, 1, &LocalizationCallback);
  ros::Subscriber goto_sub =
      n.subscribe("/move_base_simple/goal", 1, &GoToCallback); 
  ros::Subscriber server_sub = 
      n.subscribe(FLAGS_server_topic, 20, &ServerPathParamCallback);

  // File logging commands
  log_file_spin_loop << "Network Latency \n";

  // Main loop 
  RateLoop loop(10.0);
  loop.Sleep();
  while (run_ && ros::ok()) {
    std::cout << "-----------------------" << std::endl;
    ros::spinOnce();
    navigation_client_->Run();
    loop.Sleep();
    std::cout << "-----------------------" << std::endl;
  }
  delete navigation_client_;
  return 0;
}
