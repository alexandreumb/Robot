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

#include "robot_steering_controller/robot_steering_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace robot_steering_controller
{
RobotSteeringController::RobotSteeringController()
: steering_controllers_library::SteeringControllersLibrary()
{
}

void RobotSteeringController::initialize_implementation_parameter_listener()
{
  robot_param_listener_ =
    std::make_shared<robot_steering_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn RobotSteeringController::configure_odometry()
{
  robot_params_ = robot_param_listener_->get_params();

  gps_sensor_names_ = robot_params_.gps_sensor_names;
  const double front_wheels_radius = robot_params_.front_wheels_radius;
  const double rear_wheels_radius = robot_params_.rear_wheels_radius;
  const double front_wheel_track = robot_params_.front_wheel_track;
  const double rear_wheel_track = robot_params_.rear_wheel_track;
  const double wheelbase = robot_params_.wheelbase;


  if (params_.front_steering)
  {
    odometry_.set_wheel_params(rear_wheels_radius, wheelbase, rear_wheel_track);
  }
  else
  {
    odometry_.set_wheel_params(front_wheels_radius, wheelbase, front_wheel_track);
  }

  odometry_.set_odometry_type(steering_odometry::ACKERMANN_CONFIG);

  set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);

  RCLCPP_INFO(get_node()->get_logger(), "Robot odom configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool RobotSteeringController::update_odometry(const rclcpp::Duration & period)
{
  if (params_.open_loop)
  {
    odometry_.update_open_loop(last_linear_velocity_, last_angular_velocity_, period.seconds());
  }
  else
  {
    const double traction_right_wheel_value =
      state_interfaces_[STATE_TRACTION_RIGHT_WHEEL].get_value();
    const double traction_left_wheel_value =
      state_interfaces_[STATE_TRACTION_LEFT_WHEEL].get_value();
    const double steering_right_position = state_interfaces_[STATE_STEER_RIGHT_WHEEL].get_value();
    const double steering_left_position = state_interfaces_[STATE_STEER_LEFT_WHEEL].get_value();
    if (
      std::isfinite(traction_right_wheel_value) && std::isfinite(traction_left_wheel_value) &&
      std::isfinite(steering_right_position) && std::isfinite(steering_left_position))
    {
      if (params_.position_feedback)
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_position(
          traction_right_wheel_value, traction_left_wheel_value, steering_right_position,
          steering_left_position, period.seconds());
      }
      else
      {
        // Estimate linear and angular velocity using joint information
        odometry_.update_from_velocity(
          traction_right_wheel_value, traction_left_wheel_value, steering_right_position,
          steering_left_position, period.seconds());
      }
    }
  }
  return true;
}

controller_interface::InterfaceConfiguration
RobotSteeringController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(nr_state_itfs_);
  const auto traction_wheels_feedback = params_.position_feedback
                                          ? hardware_interface::HW_IF_POSITION
                                          : hardware_interface::HW_IF_VELOCITY;
  if (params_.front_steering)
  {
    for (size_t i = 0; i < rear_wheels_state_names_.size(); i++)
    {
      state_interfaces_config.names.push_back(
        rear_wheels_state_names_[i] + "/" + traction_wheels_feedback);
    }

    for (size_t i = 0; i < front_wheels_state_names_.size(); i++)
    {
      state_interfaces_config.names.push_back(
        front_wheels_state_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
    }

    for (size_t i = 0; i < gps_sensor_names_.size(); i++)
    {
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/altitude");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/latitude");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/longitude");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/velocity_x");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/velocity_y");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/heading");
    }
  }
  else
  {
    for (size_t i = 0; i < front_wheels_state_names_.size(); i++)
    {
      state_interfaces_config.names.push_back(
        front_wheels_state_names_[i] + "/" + traction_wheels_feedback);
    }

    for (size_t i = 0; i < rear_wheels_state_names_.size(); i++)
    {
      state_interfaces_config.names.push_back(
        rear_wheels_state_names_[i] + "/" + hardware_interface::HW_IF_POSITION);
    }

    for (size_t i = 0; i < gps_sensor_names_.size(); i++)
    {
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/altitude");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/latitude");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/longitude");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/velocity_x");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/velocity_y");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/heading");
    }
  }

  return state_interfaces_config;
}

controller_interface::InterfaceConfiguration
RobotSteeringController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(nr_cmd_itfs_);

  if (params_.front_steering)
  {
    for (size_t i = 0; i < params_.rear_wheels_names.size(); i++)
    {
      command_interfaces_config.names.push_back(
        params_.rear_wheels_names[i] + "/" + hardware_interface::HW_IF_VELOCITY);
    }

    for (size_t i = 0; i < params_.front_wheels_names.size(); i++)
    {
      command_interfaces_config.names.push_back(
        params_.front_wheels_names[i] + "/" + hardware_interface::HW_IF_POSITION);
    }
  }
  else
  {
    for (size_t i = 0; i < params_.front_wheels_names.size(); i++)
    {
      command_interfaces_config.names.push_back(
        params_.front_wheels_names[i] + "/" + hardware_interface::HW_IF_VELOCITY);
    }

    for (size_t i = 0; i < params_.rear_wheels_names.size(); i++)
    {
      command_interfaces_config.names.push_back(
        params_.rear_wheels_names[i] + "/" + hardware_interface::HW_IF_POSITION);
    }
  }
  return command_interfaces_config;
}
}  // namespace robot_steering_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_steering_controller::RobotSteeringController,
  controller_interface::ChainableControllerInterface)
