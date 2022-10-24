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
#include <iostream>
#include <fstream>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"

#include "navigation.h"

using amrl_msgs::Localization2DMsg;
using math_util::DegToRad;
using math_util::RadToDeg;
using navigation::Navigation;
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
DEFINE_string(laser_topic, "scan", "Name of ROS topic for LIDAR data");
//DEFINE_string(image_topic, "camera/rgb/image_raw", "Name of ROS topic for RGB image data");
//DEFINE_string(depth_topic, "/camera/depth_registered/hw_registered/image_rect", "Name of ROS topic for depth data");
DEFINE_string(rs_image_topic, "/camera/color/image_raw", "Name of the ROS topic for RGB image data from Intel realsense");
DEFINE_string(rs_depth_topic, "camera/depth/image_rect_raw", "Name of the ROS topic for depth data from Intel realsense");
DEFINE_string(odom_topic, "odom", "Name of ROS topic for odometry data");
DEFINE_string(loc_topic, "localization", "Name of ROS topic for localization");
DEFINE_string(init_topic, "initialpose", "Name of ROS topic for initialization");
DEFINE_string(map, "maps/GDC1.txt", "Name of vector map file");

bool run_ = true;
int img_num = 0;
int depth_num = 0;
sensor_msgs::LaserScan last_laser_msg_;
Navigation* navigation_ = nullptr;

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f, dt=%f\n",
           msg.header.stamp.toSec(),
           GetWallTime() - msg.header.stamp.toSec());
  }
  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(0.2, 0);
  // const Vector2f kLaserLoc(0.0, 0.0);

  vector<Vector2f> point_cloud_;
  /*
  cout << "printing the laser message" << "\n";
  cout << "minimum angle :   " << msg.angle_min << "\n";
  cout << "maximum angle :   " << msg.angle_max << "\n";
  cout << "angle increment:   " << msg.angle_increment << "\n";
  cout << "time increment:   " << msg.time_increment << "\n";
  cout << "scan time:   " <<  msg.scan_time << "\n";
  cout << "minimum range :   " << msg.range_min << "\n";
  cout << "maximum range:   " << msg.range_max << "\n";
  cout << "num laser readings : " << msg.ranges.size() << "\n";
  cout << "expected num laser readings : " << (msg.angle_max - msg.angle_min) / msg.angle_increment << "\n";
  // TODO Convert the LaserScan to a point cloud
  */
  //int a = int(msg.ranges.size());
  for(int ranges_idx = 0; ranges_idx < int(msg.ranges.size()); ranges_idx++){
    Vector2f v(0, 0);
    v[0] = msg.ranges[ranges_idx] * cos(msg.angle_min + ranges_idx * msg.angle_increment);
    v[1] = msg.ranges[ranges_idx] * sin(msg.angle_min + ranges_idx * msg.angle_increment);
    point_cloud_.push_back(v + kLaserLoc);
  }
  // The LaserScan parameters are accessible as follows:
  // msg.angle_increment // Angular increment between subsequent rays
  // msg.angle_max // Angle of the first ray
  // msg.angle_min // Angle of the last ray
  // msg.range_max // Maximum observable range
  // msg.range_min // Minimum observable range
  // msg.ranges[i] // The range of the i'th ray
  navigation_->ObservePointCloud(point_cloud_, msg.header.stamp.toSec());
  last_laser_msg_ = msg;
}


sensor_msgs::Image last_image_msg_;
void ImageCallback(const sensor_msgs::Image& msg) {
  int num_pixels = sizeof(msg.data)/sizeof(msg.data[0]);
  
  std::cout << "printing the image message" << std::endl;
  std::cout << "image height : " << msg.height;
  std::cout << "image width : " << msg.width;
  std::cout << "number of pixels : " << num_pixels;
  std::cout << "data : " << msg.data.size();

  //int height = msg.height;
  //int width = msg.width;
  //int num_channels = 3;
  //uint image[height][width][num_channels];

  std::string s = "images/" + std::to_string(img_num) + ".csv";
  cout << "filename : " << s << '\n';
  std::ofstream out(s);

  for(int didx = 0; didx < int(msg.data.size()); didx++){
    uint pixel_val = msg.data[didx];
    out << pixel_val << ',';
    out << '\n';
  }

  img_num++;

  //for(int hidx = 0; hidx < height; hidx++){
  //  for(int widx = 0; widx < width; widx++){
  //    for(int cidx = 0; cidx < num_channels; cidx++){
  //      int data_idx = hidx * width + widx * num_channels + cidx;
  //      image[hidx][widx][cidx] = msg.data[data_idx];
  //      image[hidx][widx][cidx]++; 
  //    }
  //  }
  //}

  last_image_msg_ = msg;
  //navigation_->ObserveImage(point_cloud_, msg.header.stamp.toSec());
}

sensor_msgs::Image last_depth_msg_;
void DepthCallback(const sensor_msgs::Image& msg) {
  //int num_pixels = sizeof(msg.data)/sizeof(msg.data[0]);
  std::cout << "printing the depth image message" << std::endl;
  std::cout << "depth height : " << msg.height;
  std::cout << "depth width : " << msg.width;
  //std::cout << "number of pixels : " << num_pixels;
  //std::cout << "data : " << msg.data.size();

  std::string s = "depth/" + std::to_string(img_num) + ".csv";
  cout << "filename : " << s << '\n';
  std::ofstream out(s);

  for(int didx = 0; didx < int(msg.data.size()); didx++){
    uint pixel_val = msg.data[didx];
    out << pixel_val << ',';
    out << '\n';
  }

  depth_num++;

  //for(int hidx = 0; hidx < height; hidx++){
  //  for(int widx = 0; widx < width; widx++){
  //    for(int cidx = 0; cidx < num_channels; cidx++){
  //      int data_idx = hidx * width + widx * num_channels + cidx;
  //      image[hidx][widx][cidx] = msg.data[data_idx];
  //      image[hidx][widx][cidx]++; 
  //    }
  //  }
  //}

  last_depth_msg_ = msg;


}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  navigation_->UpdateOdometry(
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
  navigation_->SetNavGoal(loc, angle);
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
  navigation_->UpdateLocation(Vector2f(msg.pose.x, msg.pose.y), msg.pose.theta);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler); 
  // Initialize ROS.
  ros::init(argc, argv, "navigation", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  navigation_ = new Navigation(FLAGS_map, &n);

  ros::Subscriber velocity_sub =     n.subscribe(FLAGS_odom_topic, 1, &OdometryCallback);
  ros::Subscriber localization_sub = n.subscribe(FLAGS_loc_topic, 1, &LocalizationCallback);
  ros::Subscriber laser_sub =        n.subscribe(FLAGS_laser_topic, 1, &LaserCallback);
  //ros::Subscriber image_sub =        n.subscribe(FLAGS_image_topic, 1, &ImageCallback);
  //ros::Subscriber depth_sub =        n.subscribe(FLAGS_depth_topic, 1, &DepthCallback);
  ros::Subscriber image_sub =        n.subscribe(FLAGS_rs_image_topic, 1, &ImageCallback);
  ros::Subscriber depth_sub =        n.subscribe(FLAGS_rs_depth_topic, 1, &DepthCallback);
  ros::Subscriber goto_sub =         n.subscribe("/move_base_simple/goal", 1, &GoToCallback); 

  RateLoop loop(20.0);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    navigation_->Run();
    loop.Sleep();
  }
  delete navigation_;
  return 0;
}
