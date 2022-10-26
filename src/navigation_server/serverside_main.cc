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
\file    serverside_main.cc
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
 
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"

#include "serverside.h"

using math_util::DegToRad;
using math_util::RadToDeg;
using serverside::Serverside;
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
DEFINE_string(map, "maps/GDC1.txt", "Name of vector map file");

bool run_ = true;
sensor_msgs::LaserScan last_laser_msg_;
Serverside* serverside_ = nullptr; //pointer to the object belonging to the Serverside class


void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f, dt=%f\n",
           msg.header.stamp.toSec(),
           GetWallTime() - msg.header.stamp.toSec());
  }
  //std::cout << "within callback" << "\n";
  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(0.2, 0);
  //cout << "sensor reading recorded time : " << msg.header.stamp << "\n";
  /*
  cout << "----------------------------" << "\n";
  cout << "printing the laser message" << "\n";
  cout << "minimum angle :   " << msg.angle_min << "\n";
  cout << "maximum angle :   " << msg.angle_max << "\n";
  cout << "angle increment:   " << msg.angle_increment << "\n";
  cout << "time increment:   " << msg.time_increment << "\n";
  cout << "scan time:   " << msg.scan_time << "\n";
  cout << "minimum range :   " << msg.range_min << "\n";
  cout << "maximum range:   " << msg.range_max << "\n";
  cout << "num laser readings : " << msg.ranges.size() << "\n";
  cout << "expected num laser readings : " << (msg.angle_max - msg.angle_min) / msg.angle_increment << "\n";
  cout << "message header : " << msg.header << "\n";
  cout << "sensor reading recorded time : " << msg.header.stamp << "\n";
  cout << "calculated sensor reading recorded time : " << h_time << "\n";
  */
  vector<Vector2f> point_cloud_;
  for(int ranges_idx = 0; ranges_idx < int(msg.ranges.size()); ranges_idx++){
    Vector2f v(0, 0);
    v[0] = msg.ranges[ranges_idx] * cos(msg.angle_min + ranges_idx * msg.angle_increment);
    v[1] = msg.ranges[ranges_idx] * sin(msg.angle_min + ranges_idx * msg.angle_increment);
    point_cloud_.push_back(v + kLaserLoc);
  }
  
  serverside_->ObservePointCloud(point_cloud_, msg.header.stamp);
  last_laser_msg_ = msg;
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler); 
  // Initialize ROS.
  ros::init(argc, argv, "navigation_server", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  serverside_ = new Serverside(FLAGS_map, &n);

  ros::Subscriber laser_sub =
      n.subscribe(FLAGS_laser_topic, 20, &LaserCallback);

  RateLoop loop(20.0);

  while (run_ && ros::ok()) {
    std::cout << "-----------------" << "\n";
    std::cout << "within while loop" << "\n";
    ros::spinOnce();
    serverside_->PopulateServersideBuffers();
    
    serverside_->Run();
    loop.Sleep();
  }

  delete serverside_;
  return 0;
}
