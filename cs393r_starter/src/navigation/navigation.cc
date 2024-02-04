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
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "motion_primitives.h"

DEFINE_bool(Test1DTOC, true, "Run 1D line time-optimal controller test");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, NavigationParams& params, ros::NodeHandle* n) :
    params_(params),
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
}

void Navigation::UpdateLocation(const Vector2f& loc, float angle) {
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

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // TODO: Predict laserscan and robot's velocity

  // TODO: Do in Local Planner
    // 1. Sample paths
    // 2. Find best path

  // TODO: Control based on the best path

  // if (FLAGS_TestStraight1DTOC) {
  //   drive_msg_.curvature = 0.0f;
  //   // Vector2f goal_loc = {odom_start_loc_[0] + 2, odom_start_loc_[1]};
  //   Vector2f goal_loc = odom_start_loc_ + Rotation2Df(odom_angle_) * Vector2f(2.0, 0);
  //   float remaining_distance_to_goal = (goal_loc - odom_loc_).norm();
  //   printf("goal loc x: %0.3f y: %0.3f", goal_loc[0], goal_loc[1]);
  //   printf("odom loc x: %0.3f y: %0.3f", odom_loc_[0], odom_loc_[1]);
  //   printf("remaining distance to goal: %0.3f", remaining_distance_to_goal);
  //   if (remaining_distance_to_goal > params_.goal_tolerance) {
  //     StraightLineTest(drive_msg_.velocity, remaining_distance_to_goal);
  //   } else {
  //     printf("Test complete");
  //     return; // Don't update anything once test is finished
  //   }
  // }

  if (FLAGS_Test1DTOC) {
    float c = -0.2f;
    float r = 1 / c;
    drive_msg_.curvature = c;
    float theta = M_PI_4;
    Vector2f goal_loc = odom_start_loc_ + Rotation2Df(odom_start_angle_) * Vector2f(fabs(r) * sin(theta), r * (1 - cos(theta)));
    float d = (goal_loc - odom_loc_).norm();
    if (d > params_.goal_tolerance) {
      float left_theta = acos((Sq(r) + Sq(r) - Sq(d)) / (2*Sq(r))); // Law of Cosines
      float left_arc_length = fabs(r * left_theta);
      ConstantCurvatureArc curve(c, left_arc_length);
      curve.getControlOnCurve(params_.linear_limits, robot_vel_.norm(), params_.dt, drive_msg_.velocity);
    } else {
      printf("Test complete");
      return; // Don't update anything once test is finished
    }
  }

  // TODO: Obstacle avoidance based on predicted laserscan and robot's velocity
  //       and local goal
  // 1. Sample path
  // 2. Cost sampled paths
  // 3. get best path
  // 4. get first cmd for best path

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

void Navigation::StraightLineTest(
  float& cmd_vel, float goal_distance) {
  /*
  This function runs a straight line test from the current location
  to the location goal_distance in front of it.
  */
  // Vector2f robot_loc_xy = odom_loc_ - odom_start_loc_;
  // Vector2f goal_loc_xy = robot_loc_xy + Rotation2Df(odom_angle_) * Vector2f(goal_distance, 0); // Rotate an x-translation to align with the robot heading to result in a straight line between the current and goal pose
  // float goal_loc = goal_loc_xy.norm();
  // float robot_loc = robot_loc_xy.norm();

  float robot_speed = robot_vel_.norm();
  cmd_vel = run1DTOC(
    params_.linear_limits, 0, robot_speed, goal_distance, 0, params_.dt
  );
  printf("d:%.3f v: %.3f cmd:%.3lf\n", goal_distance, robot_speed, cmd_vel);
}

}  // namespace navigation
