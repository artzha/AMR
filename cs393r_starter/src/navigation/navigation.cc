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

#include "navigation.h"

#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "motion_primitives.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "visualization/visualization.h"

DEFINE_bool(Test1DTOC, false, "Run 1D line time-optimal controller test");
DEFINE_bool(TestSamplePaths, true, "Run sample paths test");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
}  // namespace

namespace navigation {

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name,
                       NavigationParams& params,
                       ros::NodeHandle* n)
    : params_(params),
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
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ =
      visualization::NewVisualizationMessage("base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);

  assert(params_.obstacle_margin <
         (1.0f / params_.max_curvature - params_.robot_width) / 2);
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

void Navigation::UpdateOdometry(
    const Vector2f& loc, float angle, const Vector2f& vel, float ang_vel, double time) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  t_odom_ = time;
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

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud, double time) {
  point_cloud_ = cloud;
  t_point_cloud_ = time;
}

void Navigation::ForwardPredict(double time) {}

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
  //   Vector2f goal_loc = odom_start_loc_ + Rotation2Df(odom_angle_) * Vector2f(2.0,
  //   0); float remaining_distance_to_goal = (goal_loc - odom_loc_).norm();
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

  cout << "RUN" << endl;

  if (FLAGS_Test1DTOC) {
    test1DTOC();
  }
  if (FLAGS_TestSamplePaths) {
    testSamplePaths(drive_msg_);
  }

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

void Navigation::test1DTOC() {
  float c = -0.2f;
  float r = 1 / c;
  drive_msg_.curvature = c;
  float theta = M_PI_4;
  Vector2f goal_loc =
      odom_start_loc_ + Rotation2Df(odom_start_angle_) *
                            Vector2f(fabs(r) * sin(theta), r * (1 - cos(theta)));
  float d = (goal_loc - odom_loc_).norm();
  if (d > params_.goal_tolerance) {
    float left_theta = acos((Sq(r) + Sq(r) - Sq(d)) / (2 * Sq(r)));  // Law of Cosines
    float left_arc_length = fabs(r * left_theta);
    motion_primitives::ConstantCurvatureArc curve(c, left_arc_length);
    curve.getControlOnCurve(
        params_.linear_limits, robot_vel_.norm(), params_.dt, drive_msg_.velocity);
  } else {
    printf("Test complete");
    return;  // Don't update anything once test is finished
  }
}

void Navigation::testSamplePaths(AckermannCurvatureDriveMsg& drive_msg) {
  // Sample paths
  Vector2f local_target(20, 0);
  auto ackermann_sampler_ = motion_primitives::AckermannSampler(params_);
  ackermann_sampler_.update(robot_vel_, robot_omega_, local_target, point_cloud_);
  auto paths = ackermann_sampler_.getSamples(50);

  auto ackermann_evaluator_ = motion_primitives::AckermannEvaluator();
  auto best_path = ackermann_evaluator_.findBestPath(paths);
  best_path->getControlOnCurve(
      params_.linear_limits, robot_vel_.norm(), params_.dt, drive_msg.velocity);
  drive_msg.curvature = best_path->curvature();


  // Visualize paths
  int idx = 0;
  for (auto path : paths) {
    visualization::DrawPathOption(path->curvature(),
                                  path->arc_length(),
                                  path->clearance(),
                                  32762,
                                  false,
                                  local_viz_msg_);
    cout << "idx: " << idx++ <<  ", Curvature: " << path->curvature() << " Arc Length: " << path->arc_length()
         << " Clearance: " << path->clearance() << endl;
  }

  visualization::DrawPathOption(best_path->curvature(),
                                best_path->arc_length(),
                                best_path->clearance(),
                                10000,
                                false,
                                local_viz_msg_);

  // Visualize pointcloud
  for (auto point : point_cloud_) {
    visualization::DrawPoint(point, 32762, local_viz_msg_);
  }

  return;
}

}  // namespace navigation
