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

#include "particle_filter.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "config_reader/config_reader.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "vector_map/vector_map.h"

using Eigen::Vector2f;
using Eigen::Vector2i;
using geometry::line2f;
using std::cout;
using std::endl;
using std::fabs;
using std::min;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

// DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

CONFIG_FLOAT(num_particles_, "num_particles");
CONFIG_FLOAT(k_x_, "k_x");
CONFIG_FLOAT(k_y_, "k_y");
CONFIG_FLOAT(k_theta_, "k_theta");
CONFIG_FLOAT(k_laser_loc_x_, "k_laser_loc.x");
CONFIG_FLOAT(k_laser_loc_y_, "k_laser_loc.y");
config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter()
    : prev_odom_loc_(0, 0), prev_odom_angle_(0), odom_initialized_(false) {}

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
                                            vector<Vector2f>& scan) {
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // Group Comments: construct point cloud in world frame
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(CONFIG_k_laser_loc_x_, CONFIG_k_laser_loc_y_);

  // Construct se2 matrix for robot pose in world to transform ray line segment from
  // base_link frame to world frame se2 transformations preserve distances between
  // points
  Eigen::Affine2f T_robot = Eigen::Translation2f(loc) * Eigen::Rotation2Df(angle);

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    // define endpoint of ray line segment relative to world frame
    float theta = angle_min + i * (angle_max - angle_min) / num_ranges;
    scan[i] = T_robot *
              (Vector2f(range_max * cos(theta), range_max * sin(theta)) + kLaserLoc); // world frame
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  vector<float> ranges(num_ranges, range_max);
  for (size_t i = 0; i < scan.size(); ++i) {
    for (size_t j = 0; j < map_.lines.size(); ++j) {
      const line2f map_line = map_.lines[j];
      // Construct line segment in world frame
      line2f my_line(loc.x(), loc.y(), scan[i].x(), scan[i].y());

      // Check for intersection
      Vector2f intersection_point;  // Return variable
      bool intersects = map_line.Intersection(my_line, &intersection_point);
      if (intersects) {
        // NOTE: se2 transformations preserve distances between between points
        float new_range = (intersection_point - loc).norm();  // compute range in world frame
        if (new_range < ranges[i]) {
          scan[i] = intersection_point; // scan in world frame
          ranges[i] = new_range;
        }
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

  // int num_ranges = ranges.size();
  // for (Particle& p : particles_) {
  //   vector<Vector2f> predicted_scan(num_ranges);

  //   GetPredictedPointCloud(p.loc,
  //                          p.angle,
  //                          num_ranges,
  //                          range_min,
  //                          range_max,
  //                          angle_min,
  //                          angle_max,
  //                          predicted_scan);

  //   // Transform predicted scan to base_link frame
  // }
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

  // 1 Predict the likelihood of each particle

  // 2 Resample the particles
}

void ParticleFilter::Predict(const Vector2f& odom_loc, const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  if (!odom_initialized_) {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
    return;
  }

  printf("\n===============Predict Step===============\n");

  // U_t = X_t^-1 * X_{t+1} : transformation from prev_odom frame to current odom frame
  // Construct SE(2) matrices: rotate then translate.
  Eigen::Affine2f X_new =
      Eigen::Translation2f(odom_loc) * Eigen::Rotation2Df(odom_angle);
  Eigen::Affine2f X_old =
      Eigen::Translation2f(prev_odom_loc_) * Eigen::Rotation2Df(prev_odom_angle_);

  Eigen::Affine2f U = X_old.inverse() * X_new;  // 3x3 matrix

  // Compute Covariance of the noise
  float sigma_x = fabs(U.translation().x());
  float sigma_y = fabs(U.translation().y());
  float sigma_theta =
      fabs(atan2(U.rotation()(1, 0), U.rotation()(0, 0)));  // sin dtheta / cos dtheta

  cout << "sigma_x: " << sigma_x << " sigma_y: " << sigma_y
       << " sigma_theta: " << sigma_theta << endl;

  printf(
      "sigma_x: %.6f sigma_y: %.6f sigma_theta: %.6f", sigma_x, sigma_y, sigma_theta);
  // Predict the new pose for each particle with noise
  for (Particle& p : particles_) {
    // Convert particles to SE(2) and add noise
    // Sample from noise distributions (center at 0 by offsetting)
    float eps_x = CONFIG_k_x_ * rng_.Gaussian(0, sigma_x);
    float eps_y = CONFIG_k_y_ * rng_.Gaussian(0, sigma_y);
    float eps_theta = CONFIG_k_theta_ * rng_.Gaussian(0, sigma_theta);

    // printf("eps_x: %.6f eps_y: %.6f eps_theta: %.6f\n", eps_x, eps_y, eps_theta);
    Eigen::Affine2f e =
        Eigen::Translation2f(eps_x, eps_y) * Eigen::Rotation2Df(eps_theta);

    // particles are X^t_w
    Eigen::Affine2f particle =
        Eigen::Translation2f(p.loc) * Eigen::Rotation2Df(p.angle);

    // X^{\tilde {t+1}}_w = X^t_w * U^{t+1}_t * E^{\tilde{t+1}}_{t+1}
    Eigen::Affine2f new_particle = particle * U * e;

    // SE(2) -> x y theta
    p.loc = new_particle.translation();
    p.angle = atan2(new_particle.rotation()(1, 0), new_particle.rotation()(0, 0));
  }

  // Update the previous odometry
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  // Initialize the particles
  particles_.resize(CONFIG_num_particles_);
  for (Particle& p : particles_) {
    p.loc = loc;
    p.angle = angle;
    p.weight = 1.0 / CONFIG_num_particles_;
  }

  odom_initialized_ = false;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  loc = Vector2f(0, 0);
  angle = 0;

  for (const Particle& p : particles_) {
    loc += p.loc * p.weight;
    angle += p.angle * p.weight;
  }
  angle = math_util::AngleMod(angle);
}

}  // namespace particle_filter
