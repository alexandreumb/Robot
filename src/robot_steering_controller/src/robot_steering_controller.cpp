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

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace utility
{
using ImgAnalyzeMsg =
  robot_steering_controller::RobotSteeringController::ImgAnalyzeMsg;

// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ImgAnalyzeMsg> & msg,
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  for (size_t i = 0; i < msg->object.size(); ++i)
  {
    msg->object[i].name = "";
    msg->object[i].distance = std::numeric_limits<double>::quiet_NaN();
    msg->object[i].confidence = std::numeric_limits<double>::quiet_NaN();
  }
  msg->velocity = 0;
}

}  // namespace

namespace robot_steering_controller
{
RobotSteeringController::RobotSteeringController()
: controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn RobotSteeringController::on_init()
{
  try
  {
  robot_param_listener_ = std::make_shared<robot_steering_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotSteeringController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  robot_params_ = robot_param_listener_->get_params();

  odometry_.set_velocity_rolling_window_size(
    static_cast<size_t>(robot_params_.velocity_rolling_window_size));

  const double front_wheels_radius = robot_params_.front_wheels_radius;
  const double rear_wheels_radius = robot_params_.rear_wheels_radius;
  const double front_wheel_track = robot_params_.front_wheel_track;
  const double rear_wheel_track = robot_params_.rear_wheel_track;
  const double wheelbase = robot_params_.wheelbase;
  const double max_steering_angle = robot_params_.max_steering_angle;
  const double max_velocity = robot_params_.max_velocity;

  odometry_.set_default_params(wheelbase, max_steering_angle, max_velocity);

  nr_state_itfs_ = NR_STATE_ITFS;
  nr_cmd_itfs_ = NR_CMD_ITFS;
  nr_ref_itfs_ = NR_REF_ITFS;

  if (!robot_params_.axes_names.empty() && robot_params_.axes_names.size() == 2)
  {
    axes_names_ = robot_params_.axes_names;
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No axes names provided, cannot configure controller");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!robot_params_.direction_names.empty() && robot_params_.direction_names.size() == 1)
  {
    direction_names_ = robot_params_.direction_names;
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No direction names provided, cannot configure controller");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!robot_params_.gps_sensor_names.empty() && robot_params_.gps_sensor_names.size() == 1)
  {
    gps_sensor_names_ = robot_params_.gps_sensor_names;
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No GPS sensor names provided, cannot configure controller");
    return controller_interface::CallbackReturn::ERROR;
  }

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_timeout_ = rclcpp::Duration::from_seconds(robot_params_.reference_timeout);
  if (robot_params_.use_stamped_vel)
  {
    ref_subscriber_image_ = get_node()->create_subscription<ImgAnalyzeMsg>(
      "~/reference", subscribers_qos,
      std::bind(&RobotSteeringController::reference_callback, this, std::placeholders::_1));
  }


  std::shared_ptr<ImgAnalyzeMsg> msg =
    std::make_shared<ImgAnalyzeMsg>();
  utility::reset_controller_reference_msg(msg, get_node());
  input_ref_.writeFromNonRT(msg);

  try
  {
    // Odom state publisher
    odom_s_publisher_ = get_node()->create_publisher<ControllerStateMsgOdom>(
      "~/odometry", rclcpp::SystemDefaultsQoS());
    rt_odom_state_publisher_ = std::make_unique<ControllerStatePublisherOdom>(odom_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rt_odom_state_publisher_->lock();
  rt_odom_state_publisher_->msg_.header.stamp = get_node()->now();
  rt_odom_state_publisher_->msg_.header.frame_id = robot_params_.odom_frame_id;
  rt_odom_state_publisher_->msg_.child_frame_id = robot_params_.base_frame_id;
  rt_odom_state_publisher_->msg_.pose.pose.position.z = 0;

  auto & covariance = rt_odom_state_publisher_->msg_.twist.covariance;
  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    covariance[diagonal_index] = robot_params_.pose_covariance_diagonal[index];
    covariance[diagonal_index] = robot_params_.twist_covariance_diagonal[index];
  }
  rt_odom_state_publisher_->unlock();

  try
  {
    // Tf State publisher
    tf_odom_s_publisher_ = get_node()->create_publisher<ControllerStateMsgTf>(
      "~/tf_odometry", rclcpp::SystemDefaultsQoS());
    rt_tf_odom_state_publisher_ =
      std::make_unique<ControllerStatePublisherTf>(tf_odom_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  rt_tf_odom_state_publisher_->lock();
  rt_tf_odom_state_publisher_->msg_.transforms.resize(1);
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.stamp = get_node()->now();
  rt_tf_odom_state_publisher_->msg_.transforms[0].header.frame_id = robot_params_.odom_frame_id;
  rt_tf_odom_state_publisher_->msg_.transforms[0].child_frame_id = robot_params_.base_frame_id;
  rt_tf_odom_state_publisher_->msg_.transforms[0].transform.translation.z = 0.0;
  rt_tf_odom_state_publisher_->unlock();

  try
  {
    // State publisher
    controller_s_publisher_ = get_node()->create_publisher<SteeringControllerStateMsg>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    controller_state_publisher_ =
      std::make_unique<ControllerStatePublisher>(controller_s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  controller_state_publisher_->lock();
  controller_state_publisher_->msg_.header.stamp = get_node()->now();
  controller_state_publisher_->msg_.header.frame_id = robot_params_.odom_frame_id;
  controller_state_publisher_->unlock();
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void RobotSteeringController::reference_callback(
  const std::shared_ptr<ImgAnalyzeMsg> msg)
{
  // if no timestamp provided use current time for command timestamp
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Timestamp in header is missing, using current time as command timestamp.");
      msg->header.stamp = get_node()->now();
  }

  const auto age_of_last_command = get_node()->now() - msg->header.stamp;
  RCLCPP_INFO(get_node()->get_logger(), "Received new reference message with velocity: %f", msg->velocity);

  if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_)
  {
    input_ref_.writeFromNonRT(msg);
  }
}

bool RobotSteeringController::update_odometry(const rclcpp::Duration & period)
{
  const double heading =
    state_interfaces_[HEADING].get_value();
  const double position_x =
    state_interfaces_[POS_X].get_value();
  const double position_y =
    state_interfaces_[POS_Y].get_value();
  const double position_z =
    state_interfaces_[POS_Z].get_value();
  const double velocity_north =
    state_interfaces_[VEL_NORTH].get_value();
  const double velocity_east =
    state_interfaces_[VEL_EAST].get_value();
 
  if (id % 100 == 0)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Updating odometry with heading: %f, position_x: %f, position_y: %f, velocity_north: %f, velocity_east: %f",
      heading, position_x, position_y, velocity_north, velocity_east);
  };

  odometry_.update_from_position(
    heading, position_x, position_y, position_z,
    position_x + velocity_east * period.seconds(), position_y + velocity_north * period.seconds(), position_z, period.seconds());

  return true;
}


controller_interface::InterfaceConfiguration
RobotSteeringController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(nr_state_itfs_);
  const auto traction_wheels_feedback = robot_params_.position_feedback
                                          ? hardware_interface::HW_IF_POSITION
                                          : hardware_interface::HW_IF_VELOCITY;

  for (size_t i = 0; i < axes_names_.size(); i++)
  {
    if (axes_names_[i].find("front") != std::string::npos) {
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/temperature_front_left");  
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/temperature_front_right");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/temperature_front_timestamp");    
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/velocity_front_left");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/velocity_front_right");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/velocity_front_timestamp");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/throttle_front_left");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/throttle_front_right");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/throttle_front_timestamp");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/reverse_front_left");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/reverse_front_right");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/reverse_front_timestamp");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/brake_front_left");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/brake_front_right");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/brake_front_timestamp");
    }
    else if (axes_names_[i].find("rear") != std::string::npos) {
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/temperature_rear_left");  
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/temperature_rear_right");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/temperature_rear_timestamp");    
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/velocity_rear_left");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/velocity_rear_right");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/velocity_rear_timestamp");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/throttle_rear_left");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/throttle_rear_right");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/throttle_rear_timestamp");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/reverse_rear_left");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/reverse_rear_right");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/reverse_rear_timestamp");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/brake_rear_left");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/brake_rear_right");
      state_interfaces_config.names.push_back(
        axes_names_[i] + "/brake_rear_timestamp");
    }
    else {
      RCLCPP_ERROR(get_node()->get_logger(), "Axis name %s does not include 'front' or 'rear'", axes_names_[i].c_str());
      return controller_interface::InterfaceConfiguration();
    }
  }

  for (size_t i = 0; i < direction_names_.size(); i++)
  {
    state_interfaces_config.names.push_back(
      direction_names_[i] + "/steering_angle");
    state_interfaces_config.names.push_back(
      direction_names_[i] + "/steering_angle_timestamp");
    state_interfaces_config.names.push_back(
      direction_names_[i] + "/steering_torque_high");
    state_interfaces_config.names.push_back(
      direction_names_[i] + "/steering_torque_low");
    state_interfaces_config.names.push_back(
      direction_names_[i] + "/steering_torque_timestamp");
  }

  for (size_t i = 0; i < gps_sensor_names_.size(); i++)
  {
    state_interfaces_config.names.push_back(
      gps_sensor_names_[i] + "/height");
    state_interfaces_config.names.push_back(
      gps_sensor_names_[i] + "/latitude");
    state_interfaces_config.names.push_back(
      gps_sensor_names_[i] + "/longitude");
    state_interfaces_config.names.push_back(
      gps_sensor_names_[i] + "/velocity_north");
    state_interfaces_config.names.push_back(
      gps_sensor_names_[i] + "/velocity_east");
    state_interfaces_config.names.push_back(
      gps_sensor_names_[i] + "/heading");
  }

  return state_interfaces_config;
}

controller_interface::InterfaceConfiguration
RobotSteeringController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(nr_cmd_itfs_);

  for (size_t i = 0; i < direction_names_.size(); i++)
  {
    command_interfaces_config.names.push_back(
      direction_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY);
    command_interfaces_config.names.push_back(
      direction_names_[i] + "/direction");
  }
  
  return command_interfaces_config;
}

std::vector<hardware_interface::CommandInterface>
RobotSteeringController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(nr_ref_itfs_, std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(nr_ref_itfs_);

  reference_interfaces.push_back(
    hardware_interface::CommandInterface(
      get_node()->get_name(), std::string("linear/") + hardware_interface::HW_IF_VELOCITY,
      &reference_interfaces_[0]));

  reference_interfaces.push_back(
    hardware_interface::CommandInterface(
      get_node()->get_name(), std::string("angular/") + hardware_interface::HW_IF_VELOCITY,
      &reference_interfaces_[1]));

  return reference_interfaces;
}

controller_interface::CallbackReturn RobotSteeringController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  utility:: reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RobotSteeringController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < nr_cmd_itfs_; ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotSteeringController::update_reference_from_subscribers()
{
  // Move functionality to the `update_and_write_commands` because of the missing arguments in
  // humble - otherwise issues with multiple time-sources might happen when working with simulators

  return controller_interface::return_type::OK;
}

controller_interface::return_type RobotSteeringController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!is_in_chained_mode())
  {
    auto current_ref = *(input_ref_.readFromRT());
    if (!std::isnan(current_ref->velocity) && !std::isnan(current_ref->velocity))
    {
      reference_interfaces_[0] = current_ref->velocity;
      reference_interfaces_[1] = current_ref->velocity;
    }

  }
  if (id % 100 == 0) {
    RCLCPP_INFO(get_node()->get_logger(), "Received new reference velocity: %f", reference_interfaces_[0]);
  }
  id++;

  update_odometry(period);

// MOVE ROBOT

  // Limit velocities and accelerations:
  // TODO(destogl): add limiter for the velocities

  if (!std::isnan(reference_interfaces_[0]) && !std::isnan(reference_interfaces_[1]))
  {
    
    const auto age_of_last_command = time - (*(input_ref_.readFromRT()))->header.stamp;
    const auto timeout =
    age_of_last_command > ref_timeout_ && ref_timeout_ != rclcpp::Duration::from_seconds(0);

    // store (for open loop odometry) and set commands
    last_linear_velocity_ = timeout ? 0.0 : reference_interfaces_[0];
    last_angular_velocity_ = timeout ? 0.0 : reference_interfaces_[1];

    auto [traction_commands, steering_commands] = odometry_.get_commands(reference_interfaces_[0]);

    command_interfaces_[CMD_TRACTION_WHEELS].set_value(traction_commands);
    command_interfaces_[CMD_STEERING].set_value(steering_commands);
  }

  // Publish odometry message
  // Compute and store orientation info
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.get_heading());

  // Populate odom message and publish
  if (rt_odom_state_publisher_->trylock())
  {
    rt_odom_state_publisher_->msg_.header.stamp = time;
    rt_odom_state_publisher_->msg_.pose.pose.position.x = odometry_.get_x();
    rt_odom_state_publisher_->msg_.pose.pose.position.y = odometry_.get_y();
    rt_odom_state_publisher_->msg_.pose.pose.orientation = tf2::toMsg(orientation);
    rt_odom_state_publisher_->msg_.twist.twist.linear.x = odometry_.get_linear();
    rt_odom_state_publisher_->msg_.twist.twist.angular.z = odometry_.get_angular();
    rt_odom_state_publisher_->unlockAndPublish();
  }

  // Publish tf /odom frame
  if (robot_params_.enable_odom_tf && rt_tf_odom_state_publisher_->trylock())
  {
    rt_tf_odom_state_publisher_->msg_.transforms.front().header.stamp = time;
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.x =
      odometry_.get_x();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.translation.y =
      odometry_.get_y();
    rt_tf_odom_state_publisher_->msg_.transforms.front().transform.rotation =
      tf2::toMsg(orientation);
    rt_tf_odom_state_publisher_->unlockAndPublish();
  }

  if (controller_state_publisher_->trylock())
  {
    controller_state_publisher_->msg_.header.stamp = time;
    controller_state_publisher_->msg_.traction_wheels_position.clear();
    controller_state_publisher_->msg_.traction_wheels_velocity.clear();
    controller_state_publisher_->msg_.linear_velocity_command.clear();
    controller_state_publisher_->msg_.steer_positions.clear();
    controller_state_publisher_->msg_.steering_angle_command.clear();

    auto wheel_count = robot_params_.axes_names.size();
    wheel_count = 0;
    auto steering_node = robot_params_.direction_names.size();

    for (size_t i = 0; i < wheel_count; ++i)
    {
      if (robot_params_.position_feedback)
      {
        controller_state_publisher_->msg_.traction_wheels_position.push_back(
          state_interfaces_[i].get_value());
      }
      else
      {
        controller_state_publisher_->msg_.traction_wheels_velocity.push_back(
          state_interfaces_[i].get_value());
      }
      controller_state_publisher_->msg_.linear_velocity_command.push_back(
        command_interfaces_[i].get_value());
    }

    for (size_t i = 0; i < steering_node; ++i)
    {
      controller_state_publisher_->msg_.steer_positions.push_back(
        state_interfaces_[wheel_count + i].get_value());
      controller_state_publisher_->msg_.steering_angle_command.push_back(
        command_interfaces_[wheel_count + i].get_value());
    }

    controller_state_publisher_->unlockAndPublish();
  }

  reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();

  return controller_interface::return_type::OK;
}


}  // namespace robot_steering_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_steering_controller::RobotSteeringController,
  controller_interface::ChainableControllerInterface)
