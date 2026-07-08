// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: dr. sc. Tomislav Petkovic
 * Author: Dr. Ing. Denis Stogl
 */

#include "robot_steering_controller/steering_odometry.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <math.h>

#define RADIANS_TO_DEGREES (180.0/M_PI)

namespace steering_odometry
{
SteeringOdometry::SteeringOdometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  z_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheel_track_(0.0),
  wheelbase_(0.0),
  wheel_radius_(0.0),
  max_velocity_(0.0),
  min_velocity_(0.0),
  max_steering_angle_(0.0),
  traction_wheel_old_pos_(0.0),
  traction_right_wheel_old_pos_(0.0),
  traction_left_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_acc_(velocity_rolling_window_size),
  angular_acc_(velocity_rolling_window_size)
{
}

void SteeringOdometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  reset_accumulators();
  timestamp_ = time;
}

bool SteeringOdometry:: update_odometry(
  const double linear_velocity, const double angular_velocity, const double dt)
{
  /// Integrate odometry:
  integrate_fk(linear_velocity, angular_velocity, dt);

  /// We cannot estimate the speed with very small time intervals:
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  /// Estimate speeds using a rolling mean to filter them out:
  linear_acc_.accumulate(linear_velocity);
  angular_acc_.accumulate(angular_velocity);

  linear_ = linear_acc_.getRollingMean();
  angular_ = angular_acc_.getRollingMean();

  return true;
}

bool SteeringOdometry::update_from_position(
  const double heading, const double current_pos_x, const double current_pos_y, const double current_pos_z,
  const double next_pos_x, const double next_pos_y, const double next_pos_z, const double dt)
{
  heading_ = heading;
  x_ = current_pos_x;
  y_ = current_pos_y;
  z_ = current_pos_z;

  double theta = heading - M_PI_2;
  double alp = std::atan2(next_pos_y - current_pos_y, next_pos_x - current_pos_x) + theta;
  steer_pos_ = std::atan2(2*wheelbase_*sin(alp), 4.0) * RADIANS_TO_DEGREES;
  steer_pos_ = std::clamp(steer_pos_, -max_steering_angle_, max_steering_angle_);
}

std::tuple<double, double> SteeringOdometry::get_commands(const double velocity)
{
  linear_ = std::clamp(velocity, min_velocity_, max_velocity_);
  return {linear_, steer_pos_};
}

void SteeringOdometry::set_default_params(double wheelbase, double max_steering_angle, double max_velocity, double min_velocity)
{
  wheelbase_ = wheelbase;
  max_steering_angle_ = max_steering_angle;
  max_velocity_ = max_velocity;
  min_velocity_ = min_velocity;
}

void SteeringOdometry::set_velocity_rolling_window_size(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  reset_accumulators();
}

void SteeringOdometry::reset_odometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  reset_accumulators();
}

void SteeringOdometry::reset_accumulators()
{
  linear_acc_ = rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
  angular_acc_ = rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
}

}  // namespace steering_odometry
