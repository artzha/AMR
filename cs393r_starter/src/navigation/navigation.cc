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
#include "astar.h"
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

DEFINE_bool(simulation, false, "Run in simulation mode");
DEFINE_bool(Test1DTOC, false, "Run 1D line time-optimal controller test");
DEFINE_bool(TestSamplePaths, false, "Run sample paths test");
DEFINE_bool(TestMapLoading, false, "Run occupancy map loading test");
DEFINE_bool(TestAStar, true, "Run A* test");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
VisualizationMsg occupancy_viz_msg_;
VisualizationMsg planning_viz_msg_;
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
      autonomy_enabled_(false),
      odom_initialized_(false),
      localization_initialized_(false),
      robot_vel_(0, 0),
      robot_omega_(0),
      robot_loc_(0, 0),
      robot_angle_(0),
      nav_complete_(true),
      nav_goal_loc_(0, 0),
      nav_goal_angle_(0),
      need_plan_(false) {
  map_.Load(GetMapFileFromName(map_name));
  occ_map_ = OccupancyMap(map_.lines, params);
  occ_map_updated_ = false;

  localCarrot_ = Vector2f(0, 0);

  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 10);
  local_viz_msg_ =
      visualization::NewVisualizationMessage("base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage("map", "navigation_global");
  occupancy_viz_msg_ =
      visualization::NewVisualizationMessage("map", "navigation_occupancy");
  planning_viz_msg_ =
      visualization::NewVisualizationMessage("map", "navigation_planning");

  if (FLAGS_TestMapLoading) {
    visualization::ClearVisualizationMsg(occupancy_viz_msg_);
    occ_map_.visualization(occupancy_viz_msg_);
  }

  assert(params_.obstacle_margin <
         (1.0f / params_.max_curvature - params_.robot_width) / 2);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  need_plan_ = true;
}

void Navigation::SetAutonomy(bool autonomy_enabled) {
  autonomy_enabled_ = autonomy_enabled;

  if (!autonomy_enabled_ and command_history_.size() > 0) {
    printf("Clearing command history\n");
    command_history_.clear();  // Clear command history when autonomy is disabled
  }
}

void Navigation::UpdateLocation(const Vector2f& loc, float angle) {
  if (!localization_initialized_) {
    robot_start_loc_ = loc;
    robot_start_angle_ = angle;
    localization_initialized_ = true;
    robot_loc_ = loc;
    robot_angle_ = angle;
    return;
  }
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::PruneCommandQueue() {
  if (command_history_.empty()) return;
  const double update_time =
      min(t_odom_, t_point_cloud_);  // conservative time range for command history

  for (auto cmd_it = command_history_.begin(); cmd_it != command_history_.end();) {
    if (cmd_it->time <
        update_time - params_.dt) {  // we received another command that
                                     // is closer to our update time
                                     // anything farther in time away than 1 dt from
                                     // update time is irrelevant
      cmd_it = command_history_.erase(cmd_it);
    } else {
      ++cmd_it;
    }
  }
}

void Navigation::UpdateOdometry(
    const Vector2f& loc, float angle, const Vector2f& vel, float ang_vel, double time) {
  if (FLAGS_v > 0) {
    cout << "================ [Navigation] UPDATE ODOMETRY ================" << endl;
    cout << "loc: " << loc << " angle: " << angle << " vel: " << vel.transpose()
         << " ang_vel: " << ang_vel << " time: " << time << endl;
    cout << "==============================================================\n" << endl;
  }

  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  t_odom_ = time;
  PruneCommandQueue();

  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud, double time) {
  if (FLAGS_v > 0) {
    cout << "=============== [Navigation] UPDATE POINTCLOUD ===============" << endl;
    cout << "cloud size: " << cloud.size() << " time: " << time << endl;
    cout << "==============================================================\n" << endl;
  }
  point_cloud_ = cloud;
  t_point_cloud_ = time;
  PruneCommandQueue();

  static double last_updated_time = time;

  if (time - last_updated_time < 0.5) {
    return;
  }

  // Update occupancy map with new point cloud (cloud, tT_laser_map)
  Eigen::Affine2f T_laser_map =
      Eigen::Translation2f(robot_loc_) * Eigen::Rotation2Df(robot_angle_);

  std::vector<Vector2f> global_cloud(cloud.size());
  for (size_t i = 0; i < cloud.size(); i++) {
    global_cloud[i] = T_laser_map * cloud[i];
  }
  occ_map_.updateOccupancy(robot_loc_, global_cloud);
  occ_map_updated_ = true;
  if (FLAGS_TestMapLoading) {
    visualization::ClearVisualizationMsg(occupancy_viz_msg_);
    occ_map_.visualization(occupancy_viz_msg_);
  }

  last_updated_time = time;
}

void Navigation::UpdateCommandHistory(const AckermannCurvatureDriveMsg& drive_msg) {
  Command new_cmd;
  new_cmd.time = ros::Time::now().toSec() + params_.system_latency;
  new_cmd.drive_msg = drive_msg;

  command_history_.push_back(new_cmd);
}

void Navigation::ForwardPredict(double time) {
  if (FLAGS_v > 1) {
    cout << "================ [Navigation] FORWARD PREDICT ================" << endl;
    cout << "forward time: " << time << endl;
  }

  Eigen::Affine2f fp_odom_tf =
      Eigen::Translation2f(odom_loc_) * Eigen::Rotation2Df(odom_angle_);
  Eigen::Affine2f fp_local_tf = Eigen::Affine2f::Identity();
  for (const Command& cmd : command_history_) {
    const float cmd_v = cmd.drive_msg.velocity;
    const float cmd_omega = cmd.drive_msg.velocity * cmd.drive_msg.curvature;
    // Assume constant velocity and omega over the time interval
    // want to compute distance traveled (vel * dt) and arc length (ang_vel * dt)
    // and the value of the linear and angular velocity to use gives us the area
    // of the trapezoid in the v-t space when we multiply by dt
    const float v_mid = (robot_vel_.norm() + cmd_v) / 2.0f;
    const float omega_mid = (robot_omega_ + cmd_omega) / 2.0f;

    // Forward Predict Odometry
    if (cmd.time >=
        t_odom_ - params_.dt) {  // start forward integrating from command before our
                                 // most recent odometry (we are still executing the
                                 // command in the timestep before the odom message)
      // we know that this gives us the last command because the controller runs at a
      // fixed frequency and you generate one command per controller iteration every dt

      // in every iteration besides the first command in the control history, t_odom_ <
      // cmd.time
      const double dt =
          (t_odom_ >
           cmd.time)  // find time that we are executing the previous command for
              ? min<double>(params_.dt - (t_odom_ - cmd.time), params_.dt)
              : min<double>(
                    time - cmd.time,
                    params_.dt);  // upper bound on how long we execute a command

      // see trapezoid comment above for v_mid and omega_mid
      const float dtheta = omega_mid * dt;
      const float ds = v_mid * dt;
      // Translation coeff for exponential map of se2 -- lie algebra dead reckoning /
      // autonomous error equation: does not estimate future state based on past state
      // Special matrix that tells you how much the angular rate of rotation affects the
      // translation because we integrate the linear velocity and have angular rotation
      // unless we are moving straight, we accummulate error by just using the linear
      // velocity (the tangent -- it is a linear approximation) especially if our
      // curvature is high a smaller dt may not necessarily decrease error with the
      // linear velocity approximation; we could increase and decrease our error between
      // steps
      Eigen::Matrix2f V = Eigen::Matrix2f::Identity();
      if (fabs(dtheta) > kEpsilon) {
        V << sin(dtheta) / dtheta, (cos(dtheta) - 1) / dtheta,
            (1 - cos(dtheta)) / dtheta, sin(dtheta) / dtheta;
      }
      // Exponential map of translation part of se2 (in local frame)
      Eigen::Vector2f dloc = V * Eigen::Vector2f(ds, 0);
      // Update odom_tf in odom frame
      fp_odom_tf = fp_odom_tf * Eigen::Translation2f(dloc) * Eigen::Rotation2Df(dtheta);
      // Update odom_loc_ and odom_angle_ in odom frame
      fp_odom_tf = fp_odom_tf * Eigen::Translation2f(dloc) * Eigen::Rotation2Df(dtheta);
      odom_loc_ = fp_odom_tf.translation();
      odom_angle_ = atan2(fp_odom_tf(1, 0), fp_odom_tf(0, 0));

      // Update robot_vel_ and robot_omega_
      robot_vel_ = Eigen::Rotation2Df(odom_angle_) * Vector2f(cmd_v, 0);
      robot_omega_ = cmd_omega;
      t_odom_ = cmd.time;  // keep track of time of command we last executed

      if (FLAGS_v > 1) {
        cout << "dtheta " << dtheta << ", ds " << ds << endl;
        cout << "V \n" << V << endl;
        cout << "odom_loc_: " << odom_loc_.transpose()
             << ", odom_angle_: " << odom_angle_ << endl;
      }
    }

    // Forward Predict Point Cloud
    if (cmd.time >= t_point_cloud_ - params_.dt) {
      const double dt =
          (t_point_cloud_ > cmd.time)
              ? min<double>(params_.dt - (t_point_cloud_ - cmd.time), params_.dt)
              : min<double>(time - cmd.time, params_.dt);
      // cout << "dt " << dt << endl;
      // cout << "params_.dt " << params_.dt << endl;
      // cout << "t_point_cloud_ " << t_point_cloud_ << " cmd.time " << cmd.time <<
      // endl;
      const float dtheta = omega_mid * dt;
      const float ds = v_mid * dt;
      // Translation coeff for exponential map of se2
      Eigen::Matrix2f V = Eigen::Matrix2f::Identity();
      if (fabs(dtheta) > kEpsilon) {
        V << sin(dtheta) / dtheta, (cos(dtheta) - 1) / dtheta,
            (1 - cos(dtheta)) / dtheta, sin(dtheta) / dtheta;
      }
      // cout << "V" << V << endl;
      // Exponential map of translation part of se2 (in local frame)
      Eigen::Vector2f dloc = V * Eigen::Vector2f(ds, 0);
      // Update local_tf in local frame
      fp_local_tf =
          fp_local_tf * Eigen::Translation2f(dloc) * Eigen::Rotation2Df(dtheta);
    }
  }

  // Forward Predict Point Cloud
  Eigen::Affine2f inv_fp_local_tf = fp_local_tf.inverse();
  fp_point_cloud_.resize(point_cloud_.size());
  for (size_t i = 0; i < point_cloud_.size(); i++) {
    fp_point_cloud_[i] = inv_fp_local_tf * point_cloud_[i];
  }
}

void Navigation::Plan() {
  std::cout << "============= [Navigation] Plan =============" << std::endl;
  std::cout << "Start loc: " << robot_loc_.transpose() << std::endl;
  std::cout << "Goal loc: " << nav_goal_loc_.transpose() << std::endl;
  visualization::ClearVisualizationMsg(planning_viz_msg_);

  AStar astar(occ_map_);
  path_ = astar.findPath(robot_loc_, nav_goal_loc_);

  if (path_.empty()) {
    std::cout << "No path found" << std::endl;
    return;
  }

  // Select Waypoints uniformly along path, making sure to always include last point
  waypoints_.clear();
  float min_dist_waypoints = params_.waypoints_coeff * 1.0f / params_.max_curvature;

  float accumulatedDistance = 0.0f;  // Initialize accumulated distance
  Eigen::Vector2f lastWaypoint = path_.front();
  for (size_t i = 1; i < path_.size(); ++i) {
    accumulatedDistance += (path_[i] - path_[i - 1]).norm();

    if (accumulatedDistance > min_dist_waypoints) {
      waypoints_.push_back(path_[i]);
      accumulatedDistance = 0.0f;  // Reset accumulated distance
      lastWaypoint = path_[i];     // Update the last added waypoint
      visualization::DrawCross(path_[i], 0.1, 62762, planning_viz_msg_);
    }
  }
  // Ensure the last point is always included, if not already the last added waypoint
  if (lastWaypoint != path_.back()) {
    waypoints_.push_back(path_.back());
    visualization::DrawCross(path_.back(), 0.1, 62762, planning_viz_msg_);
  }

  astar.visualization(planning_viz_msg_);  // Visualize the A* search (optional)
  currCarrotIdx_ = 0;
}

RunState Navigation::updateCarrot() {
  // Loop through carrots until we find one that meets the following conditions
  // 1. The carrot is kinematically feasible
  // 2. current robot location is within a certain distance of the carrot
  //      - use larger error distance for intermediate carrots
  //      - use smaller error distance for final carrot
  if (waypoints_.empty()) {
    std::cout << "No waypoints to follow" << std::endl;
    return RunState::IDLE;
  }

  // Computations are in robot frame
  // Check if the current carrot is kinematically feasible
  const float min_radius = 1.0 / params_.max_curvature;
  const Vector2f leftICmin(0, min_radius);
  const Vector2f rightICmin(0, -min_radius);

  visualization::DrawCross(
      leftICmin, 0.1, visualization::RGB2INT(255, 255, 0), local_viz_msg_);
  visualization::DrawCross(
      rightICmin, 0.1, visualization::RGB2INT(255, 255, 0), local_viz_msg_);

  Eigen::Affine2f robot_tf =
      Eigen::Translation2f(robot_loc_) * Eigen::Rotation2Df(robot_angle_);
  Eigen::Affine2f extrinsic_tf = robot_tf.inverse();

  while (currCarrotIdx_ < static_cast<int>(waypoints_.size())) {
    localCarrot_ = extrinsic_tf * waypoints_[currCarrotIdx_];

    // Case 1: Kinematically infeasible
    bool kinematically_feasible = (localCarrot_ - leftICmin).norm() > min_radius &&
                                  (localCarrot_ - rightICmin).norm() > min_radius;

    // Case 2: Close enough to current carrot
    bool waypoint_is_goal = currCarrotIdx_ == static_cast<int>(waypoints_.size()) - 1;
    bool is_far_enough = waypoint_is_goal
                             ? localCarrot_.norm() > params_.goal_distance
                             : localCarrot_.norm() > params_.carrot_distance;

    // If either case is true, use current carrot
    if (kinematically_feasible && is_far_enough) {
      visualization::DrawCross(waypoints_[currCarrotIdx_],
                               0.3,
                               visualization::RGB2INT(255, 0, 0),
                               planning_viz_msg_);
      visualization::DrawCross(
          localCarrot_, 0.5, visualization::RGB2INT(0, 255, 255), local_viz_msg_);

      return RunState::RUNNING;
    }

    ++currCarrotIdx_;
  }

  // Should only reach here once we have reached the final carrot
  std::cout << "Reached final carrot" << std::endl;
  return RunState::GOAL_REACHED;
}

void Navigation::Run() {
  static double t_last = 0;
  if (GetMonotonicTime() - t_last >= 1) {  // Publish vis messages at 1 hz
    // viz_pub_.publish(occupancy_viz_msg_);  // Rate-limit visualization.
    viz_pub_.publish(planning_viz_msg_);
    t_last = GetMonotonicTime();
  }

  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) {
    return;
  }

  if (need_plan_) {
    Plan();
    need_plan_ = false;
  }

  if (FLAGS_simulation || autonomy_enabled_) {
    // Predict laserscan and robot's odom state
    ForwardPredict(ros::Time::now().toSec() + params_.system_latency);

    // If we have waypoints, check if path is clear
    if (!waypoints_.empty() && occ_map_updated_) {
      for (const Vector2f& waypoint : path_) {
        if (occ_map_.isOccupied(waypoint)) {
          need_plan_ = true;
          occ_map_updated_ = false;
          break;
        }
      }
    }

    // Pick the next carroy
    RunState state = updateCarrot();

    if (state == RunState::GOAL_REACHED || state == RunState::IDLE) {
      drive_msg_.velocity = 0;
      drive_msg_.curvature = 0;
    } else {
      // 1DTOC with the converted carrot
      followCarrot(drive_msg_);
    }

    // if (FLAGS_Test1DTOC) {
    //   test1DTOC();
    // }
    // if (FLAGS_TestSamplePaths) {
    //   testSamplePaths(drive_msg_);
    // }

    drive_msg_.header.stamp = ros::Time::now();
    drive_pub_.publish(drive_msg_);
    // Queue the current command for future comparison.
    UpdateCommandHistory(drive_msg_);
  }

  // Visualize pointcloud
  for (auto point : fp_point_cloud_) {
    visualization::DrawPoint(point, 32762, local_viz_msg_);  // 32762
  }

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();

  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
}  // namespace navigation

void Navigation::followCarrot(AckermannCurvatureDriveMsg& drive_msg) {
  auto ackermann_sampler_ = motion_primitives::AckermannSampler(params_);
  ackermann_sampler_.update(robot_vel_, robot_omega_, localCarrot_, point_cloud_);
  auto paths = ackermann_sampler_.getSamples(50);

  auto ackermann_evaluator_ = motion_primitives::AckermannEvaluator(params_);
  ackermann_evaluator_.update(localCarrot_);
  auto best_path = ackermann_evaluator_.findBestPath(paths);
  best_path->getControlOnCurve(
      params_.linear_limits, robot_vel_.norm(), params_.dt, drive_msg.velocity);
  drive_msg.curvature = best_path->curvature();

  // Visualize paths
  // int idx = 0;
  for (auto path : paths) {
    visualization::DrawPathOption(path->curvature(),
                                  path->arc_length(),
                                  path->clearance(),
                                  32762,
                                  false,
                                  local_viz_msg_);
    // cout << "idx: " << idx++ <<  ", Curvature: " << path->curvature() << " Arc
    // Length: " << path->arc_length()
    //      << " Clearance: " << path->clearance() << endl;
  }

  visualization::DrawPathOption(best_path->curvature(),
                                best_path->arc_length(),
                                best_path->clearance(),
                                10000,
                                false,
                                local_viz_msg_);

  return;
}

void Navigation::test1DTOC() {
  float c = 0;
  // float r = 1 / c;
  float r = 100;
  drive_msg_.curvature = c;
  float theta = M_PI_4;
  Vector2f goal_loc =
      robot_start_loc_ + Rotation2Df(robot_start_angle_) *
                             Vector2f(fabs(r) * sin(theta), r * (1 - cos(theta)));

  // Vector2f goal_loc_in_base_link_ = goal_loc - odom_loc_;
  visualization::DrawCross(Vector2f(0, 0), 0.5, 32762, global_viz_msg_);
  visualization::DrawCross(goal_loc, 0.1, 32762, global_viz_msg_);
  printf("Running 1D TOC test\n");
  printf("odom start loc x: %0.3f y: %0.3f theta:%0.3f\n",
         odom_start_loc_[0],
         odom_start_loc_[1],
         odom_start_angle_);
  printf("robot start loc x: %0.3f y: %0.3f theta %0.3f",
         robot_start_loc_[0],
         robot_start_loc_[1],
         robot_start_angle_);
  printf("robot loc x: %0.3f y: %0.3f theta %0.3f",
         robot_loc_[0],
         robot_loc_[1],
         robot_angle_);
  printf("odom loc x: %0.3f y: %0.3f theta:%0.3f\n",
         odom_loc_[0],
         odom_loc_[1],
         odom_angle_);
  printf("goal loc x: %0.3f y: %0.3f\n", goal_loc[0], goal_loc[1]);
  float d = (goal_loc - robot_loc_).norm();
  if (d > params_.goal_tolerance) {
    float left_theta = acos((Sq(r) + Sq(r) - Sq(d)) / (2 * Sq(r)));  // Law of Cosines
    float left_arc_length = fabs(r * left_theta);
    motion_primitives::ConstantCurvatureArc curve(c, left_arc_length);
    curve.getControlOnCurve(
        params_.linear_limits, robot_vel_.norm(), params_.dt, drive_msg_.velocity);
  } else {
    drive_msg_.velocity = 0;
    printf("Test complete\n");
    return;  // Don't update anything once test is finished
  }
}

void Navigation::testSamplePaths(AckermannCurvatureDriveMsg& drive_msg) {
  // Sample paths

  printf("odom loc x: %0.3f y: %0.3f theta:%0.3f\n",
         odom_loc_[0],
         odom_loc_[1],
         odom_angle_);

  Vector2f local_target(7, 0);
  auto ackermann_sampler_ = motion_primitives::AckermannSampler(params_);
  ackermann_sampler_.update(robot_vel_, robot_omega_, local_target, point_cloud_);
  auto paths = ackermann_sampler_.getSamples(50);

  auto ackermann_evaluator_ = motion_primitives::AckermannEvaluator(params_);
  ackermann_evaluator_.update(local_target);
  auto best_path = ackermann_evaluator_.findBestPath(paths);
  best_path->getControlOnCurve(
      params_.linear_limits, robot_vel_.norm(), params_.dt, drive_msg.velocity);
  drive_msg.curvature = best_path->curvature();

  // Visualize paths
  // int idx = 0;
  for (auto path : paths) {
    visualization::DrawPathOption(path->curvature(),
                                  path->arc_length(),
                                  path->clearance(),
                                  32762,
                                  false,
                                  local_viz_msg_);
    // cout << "idx: " << idx++ <<  ", Curvature: " << path->curvature() << " Arc
    // Length: " << path->arc_length()
    //      << " Clearance: " << path->clearance() << endl;
  }

  visualization::DrawPathOption(best_path->curvature(),
                                best_path->arc_length(),
                                best_path->clearance(),
                                10000,
                                false,
                                local_viz_msg_);

  return;
}

}  // namespace navigation
