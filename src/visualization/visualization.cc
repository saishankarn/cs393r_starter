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
\file    visualization.cc
\brief   Helper functions for visualizations
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <string>

#include "eigen3/Eigen/Dense"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/ColoredArc2D.h"
#include "amrl_msgs/ColoredLine2D.h"
#include "amrl_msgs/PathVisualization.h"
#include "amrl_msgs/ColoredPoint2D.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "ros/ros.h"

#include "visualization.h"

using Eigen::Vector2f;
using amrl_msgs::ColoredArc2D;
using amrl_msgs::ColoredLine2D;
using amrl_msgs::ColoredPoint2D;
using amrl_msgs::Pose2Df;
using amrl_msgs::PathVisualization;
using amrl_msgs::VisualizationMsg;
using std::string;

namespace {
template <class T1, class T2>
void SetPoint(const T1& p1, T2* p2) {
  p2->x = p1.x();
  p2->y = p1.y();
}

}  // namespace

namespace visualization {

// Clear all elements in the message.
void ClearVisualizationMsg(VisualizationMsg& msg) {
  msg.particles.clear();
  msg.path_options.clear();
  msg.points.clear();
  msg.lines.clear();
  msg.arcs.clear();
}

// Return new visualization message, with initialized headers and namespace.
VisualizationMsg NewVisualizationMessage(
    const string& frame, const string& ns) {
  VisualizationMsg msg;
  msg.header.frame_id = frame;
  msg.header.seq = 0;
  msg.ns = ns;
  return msg;
}

void DrawPoint(const Vector2f& p, uint32_t color, VisualizationMsg& msg) {
  ColoredPoint2D point;
  SetPoint(p, &point.point);
  point.color = color;
  msg.points.push_back(point);
}

void DrawLine(const Vector2f& p0,
              const Vector2f& p1,
              uint32_t color,
              VisualizationMsg& msg) {
  ColoredLine2D line;
  SetPoint(p0, &line.p0);
  SetPoint(p1, &line.p1);
  line.color = color;
  msg.lines.push_back(line);
}

void DrawCross(const Eigen::Vector2f& location,
               float size,
               uint32_t color,
               VisualizationMsg& msg) {
  DrawLine(location + Vector2f(size, size),
           location - Vector2f(size, size),
           color,
           msg);
  DrawLine(location + Vector2f(size, -size),
           location - Vector2f(size, -size),
           color,
           msg);
}

void DrawArc(const Vector2f& center,
             float radius,
             float start_angle,
             float end_angle,
             uint32_t color,
             VisualizationMsg& msg) {
  ColoredArc2D arc;
  SetPoint(center, &arc.center);
  arc.radius = radius;
  arc.start_angle = start_angle;
  arc.end_angle = end_angle;
  arc.color = color;
  msg.arcs.push_back(arc);
}

void DrawParticle(const Vector2f& loc,
                  float angle,
                  VisualizationMsg& msg) {
  Pose2Df particle;
  particle.x = loc.x();
  particle.y = loc.y();
  particle.theta = angle;
  msg.particles.push_back(particle);
}

void DrawPathOption(const float curvature,
                    const float distance,
                    const float clearance,
                    VisualizationMsg& msg) {
  PathVisualization option;
  option.curvature = curvature;
  option.distance = distance;
  option.clearance = clearance; 
  msg.path_options.push_back(option);
}

void DrawRobotMargin(const float l,
                     const float w,
                     const float b,
                     const float t,
                     const float m,
                     amrl_msgs::VisualizationMsg& msg) {
  Eigen::Vector2f p1(-(l-b)/2 - m,  (w/2 + m));
  Eigen::Vector2f p2( (l+b)/2 + m,  (w/2 + m));
  Eigen::Vector2f p3( (l+b)/2 + m, -(w/2 + m));                    
  Eigen::Vector2f p4(-(l-b)/2 - m, -(w/2 + m));
  DrawLine(p1, p2, 0x000000, msg);
  DrawLine(p2, p3, 0x000000, msg);
  DrawLine(p3, p4, 0x000000, msg);
  DrawLine(p4, p1, 0x000000, msg);
}

  void DrawPath(float curvature, 
                          float distance, 
                          uint32_t color,
                          amrl_msgs::VisualizationMsg& msg) {
    if (fabs(curvature) > 1e-3) {
      float direction_of_turning = (signbit(curvature) ?  -1 : 1); //1: left turn, -1: right turn
      if (direction_of_turning > 0)
        DrawArc({0, 1/curvature}, fabs(1/curvature), -M_PI/2, 
                -M_PI/2 + distance*fabs(curvature), color, msg);
      else
        DrawArc({0, 1/curvature}, fabs(1/curvature), M_PI/2 - distance*fabs(curvature), 
                M_PI/2, color, msg);
    }
    else 
      DrawLine(Vector2f(0.0, 0.0), Vector2f(distance, 0.0), color, msg);
  }

}  // namespace visualization-