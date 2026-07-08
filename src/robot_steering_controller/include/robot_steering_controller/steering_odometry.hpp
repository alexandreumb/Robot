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
//
// Authors: dr. sc. Tomislav Petkovic, Dr. Ing. Denis Štogl
//

#ifndef ROBOT_STEERING_CONTROLLER__STEERING_ODOMETRY_HPP_
#define ROBOT_STEERING_CONTROLLER__STEERING_ODOMETRY_HPP_

#include <cmath>
#include <tuple>
#include <vector>

#include <rclcpp/time.hpp>
#include "rcppmath/rolling_mean_accumulator.hpp"

namespace steering_odometry
{
const unsigned int ROBOT_CONFIG = 0;

inline bool is_close_to_zero(double val) { return std::fabs(val) < 1e-6; }

/**
 * \brief The Odometry class handles odometry readings
 * (2D pose and velocity with related timestamp)
 */
class SteeringOdometry
{
public:
  /**
   * \brief Constructor
   * Timestamp will get the current time value
   * Value will be set to zero
   * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
   *
   */
  explicit SteeringOdometry(size_t velocity_rolling_window_size = 10);

  /**
   * \brief Initialize the odometry
   * \param time Current time
   */
  void init(const rclcpp::Time & time);

  /**
   * \brief Updates the odometry class with latest robot position
   * \param heading Robot heading [rad]
   * \param current_pos_x Current x position [m]
   * \param current_pos_y Current y position [m]
   * \param current_pos_z Current z position [m]
   * \param next_pos_x Next x position [m]
   * \param next_pos_y Next y position [m]
   * \param next_pos_z Next z position [m]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
bool update_from_position(
  const double heading, const double current_pos_x, const double current_pos_y, const double current_pos_z,
  const double next_pos_x, const double next_pos_y, const double next_pos_z, const double dt);

  /**
   * \brief heading getter
   * \return heading [rad]
   */
  double get_heading() const { return heading_; }

  /**
   * \brief x position getter
   * \return x position [m]
   */
  double get_x() const { return x_; }

  /**
   * \brief y position getter
   * \return y position [m]
   */
  double get_y() const { return y_; }

  /**
   * \brief linear velocity getter
   * \return linear velocity [m/s]
   */
  double get_linear() const { return linear_; }

  /**
   * \brief angular velocity getter
   * \return angular velocity [rad/s]
   */
  double get_angular() const { return angular_; }

  /**
   * \brief Sets default parameters: wheelbase and maximum steering angle
   * \param wheelbase Wheelbase of the robot [m]
   * \param max_steering_angle Maximum steering angle [degrees]
   * \param max_velocity Maximum velocity [m/s]
   */

  void set_default_params(double wheelbase, double max_steering_angle, double max_velocity, double min_velocity);

  /**
   * \brief Velocity rolling window size setter
   * \param velocity_rolling_window_size Velocity rolling window size
   */
  void set_velocity_rolling_window_size(const size_t velocity_rolling_window_size);

  /**
   * \brief Get velocity and steering commands based on the current odometry and the desired velocity
   * \param velocity Desired linear velocity [m/s]
   * \return Tuple of velocity commands and steering commands
   */
  std::tuple<double, double> get_commands(const double velocity);

  /**
   *  \brief Reset poses, heading, and accumulators
   */
  void reset_odometry();

private:
  /**
   * \brief Uses precomputed linear and angular velocities to compute odometry
   * \param v_bx  Linear  velocity   [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt      time difference to last call
   */
  bool update_odometry(const double v_bx, const double omega_bz, const double dt);

  /**
   * \brief Integrates the velocities (linear and angular)
   * \param v_bx Linear velocity [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt time difference to last call
   */
  void integrate_fk(const double v_bx, const double omega_bz, const double dt);

  /**
   *  \brief Reset linear and angular accumulators
   */
  void reset_accumulators();

  /// Current timestamp:
  rclcpp::Time timestamp_;

  /// Current pose:
  double x_;          //   [m]
  double y_;          //   [m]
  double z_;          //   [m]
  double steer_pos_;  // [rad]
  double heading_;    // [rad]
  double max_steering_angle_; // [degrees]

  /// Current velocity:
  double linear_;   //   [m/s]
  double angular_;  // [rad/s]
  double max_velocity_; // [m/s]
  double min_velocity_; // [m/s]

  /// Kinematic parameters
  double wheel_track_;   // [m]
  double wheelbase_;     // [m]
  double wheel_radius_;  // [m]

  /// Configuration type used for the forward kinematics
  int config_type_ = -1;

  /// Previous wheel position/state [rad]:
  double traction_wheel_old_pos_;
  double traction_right_wheel_old_pos_;
  double traction_left_wheel_old_pos_;
  /// Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  rcppmath::RollingMeanAccumulator<double> linear_acc_;
  rcppmath::RollingMeanAccumulator<double> angular_acc_;
};
}  // namespace steering_odometry

#endif  // ROBOT_STEERING_CONTROLLER__STEERING_ODOMETRY_HPP_
