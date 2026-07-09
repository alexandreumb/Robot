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
  using TeleopCommand =
    robot_steering_controller::RobotSteeringController::TeleopCommand;

  // called from RT control loop
  void reset_controller_reference_image_msg(
    const std::shared_ptr<ImgAnalyzeMsg> & msg,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
  {
    for (size_t i = 0; i < msg->object.size(); ++i)
    {
      msg->object[i].label = std::numeric_limits<double>::quiet_NaN();
      msg->object[i].probability = std::numeric_limits<double>::quiet_NaN();
      msg->object[i].box.x = std::numeric_limits<double>::quiet_NaN();
      msg->object[i].box.y = std::numeric_limits<double>::quiet_NaN();
      msg->object[i].box.width = std::numeric_limits<double>::quiet_NaN();
      msg->object[i].box.height = std::numeric_limits<double>::quiet_NaN();
      msg->object[i].kps.clear();
      msg->object[i].point.pose_x = std::numeric_limits<double>::quiet_NaN();
      msg->object[i].point.pose_y = std::numeric_limits<double>::quiet_NaN();
      msg->object[i].point3d.x = std::numeric_limits<double>::quiet_NaN();
      msg->object[i].point3d.y = std::numeric_limits<double>::quiet_NaN();
      msg->object[i].point3d.z = std::numeric_limits<double>::quiet_NaN();
    }
    msg->header.stamp = node->now();
    msg->has_object = 0;
  }

  void reset_controller_reference_teleopcommand_msg(
    const std::shared_ptr<TeleopCommand> & msg,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
  {
    msg->velocity = std::numeric_limits<double>::quiet_NaN();
    msg->angle = std::numeric_limits<double>::quiet_NaN();
    msg->header.stamp = node->now();
  }
} 

namespace robot_steering_controller
{
RobotSteeringController::RobotSteeringController()
: controller_interface::ControllerInterface()
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
    const double max_steering_angle = robot_params_.max_steering_angle;
    const double max_velocity = robot_params_.max_velocity;
    const double min_velocity = robot_params_.min_velocity;
    wheelbase_ = robot_params_.wheelbase;
    stop_distance_ = robot_params_.stop_distance;
    slow_distance_ = robot_params_.slow_distance;

    odometry_.set_default_params(wheelbase_, max_steering_angle, max_velocity, min_velocity);

    nr_state_itfs_ = NR_STATE_ITFS;
    nr_cmd_itfs_ = NR_CMD_ITFS;

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

      ref_subscriber_teleopcommand_ = get_node()->create_subscription<TeleopCommand>(
        "~/reference_teleop", subscribers_qos,
        std::bind(&RobotSteeringController::reference_teleop, this, std::placeholders::_1));
    }

    std::shared_ptr<ImgAnalyzeMsg> msg_img =
      std::make_shared<ImgAnalyzeMsg>();
    utility::reset_controller_reference_image_msg(msg_img, get_node());
    input_ref_img_.writeFromNonRT(msg_img);

    std::shared_ptr<TeleopCommand> msg_teleop =
      std::make_shared<TeleopCommand>();
    utility::reset_controller_reference_teleopcommand_msg(msg_teleop, get_node());
    input_ref_teleop_.writeFromNonRT(msg_teleop);

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

  bool RobotSteeringController::update_odometry(const rclcpp::Duration & period)
  {
    const double heading =
      state_interfaces_[HEADING].get_value();
    const double position_x =
      state_interfaces_[LONGITUDE].get_value();
    const double position_y =
      state_interfaces_[LATITUDE].get_value();
    const double position_z =
      state_interfaces_[HEIGHT].get_value();
    const double next_position_x =
      state_interfaces_[NEXT_POS_X].get_value();
    const double next_position_y =
      state_interfaces_[NEXT_POS_Y].get_value();
    const double next_position_z =
      state_interfaces_[NEXT_POS_Z].get_value();
    const double velocity_north =
      state_interfaces_[VEL_NORTH].get_value();
    const double velocity_east =
      state_interfaces_[VEL_EAST].get_value();
  
    if (tracked_object_id_ % 100 == 0)
    {
      //RCLCPP_INFO(get_node()->get_logger(), "Updating odometry with heading: %f, position_x: %f, position_y: %f, velocity_north: %f, velocity_east: %f",
      //  heading, position_x, position_y, velocity_north, velocity_east);
      //RCLCPP_INFO(get_node()->get_logger(), "Next position: x: %f, y: %f, z: %f", next_position_x, next_position_y, next_position_z);
    };

    odometry_.update_from_position(
      heading, position_x, position_y, position_z,
      next_position_x, next_position_y, next_position_z, period.seconds());

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
        gps_sensor_names_[i] + "/longitude");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/latitude");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/height");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/velocity_north");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/velocity_east");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/heading");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/next_point_north");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/next_point_east");
      state_interfaces_config.names.push_back(
        gps_sensor_names_[i] + "/next_point_down");  
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

  controller_interface::CallbackReturn RobotSteeringController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    utility::reset_controller_reference_image_msg(*(input_ref_img_.readFromRT()), get_node());
    utility::reset_controller_reference_teleopcommand_msg(*((input_ref_teleop_.readFromRT())), get_node());
    reference_velocity_ = 0.0;
    reference_angle_ = 0.0;

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn RobotSteeringController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
  {
    halt();
    utility::reset_controller_reference_image_msg(*(input_ref_img_.readFromRT()), get_node());
    utility::reset_controller_reference_teleopcommand_msg(*((input_ref_teleop_.readFromRT())), get_node());
    last_velocity_ = std::numeric_limits<double>::quiet_NaN();
    last_angle_ = std::numeric_limits<double>::quiet_NaN();
    reference_velocity_ = std::numeric_limits<double>::quiet_NaN();
    reference_angle_ = std::numeric_limits<double>::quiet_NaN();

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type RobotSteeringController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    auto current_data = *(input_ref_teleop_.readFromRT());
    auto current_objects = *(input_ref_img_.readFromRT());

    //const auto t = time - (*(input_ref_img_.readFromRT()))->middle_header.stamp;
    //RCLCPP_INFO(get_node()->get_logger(), "Update middle: %f seconds", t.seconds());
    const auto age_of_last_command = time - (*(input_ref_img_.readFromRT()))->header.stamp; 
    //RCLCPP_INFO(get_node()->get_logger(), "Update 1: %f seconds", age_of_last_command.seconds());

    if (current_objects->has_object == 1)
    {
      const auto timeout =
      age_of_last_command > ref_timeout_ && ref_timeout_ != rclcpp::Duration::from_seconds(0);
      if (!timeout)
      {
        object_tracking_ +=1;
        for (int i = 0; i < current_objects->object.size(); ++i)
        {
          car_movement = object_detection(current_objects->object[i].label,
          current_objects->object[i].point3d.x,
          current_objects->object[i].point3d.y,
          current_objects->object[i].point3d.z);

          if (car_movement == EMERGENCY_STOP)
          {
            halt_ = car_movement;
            RCLCPP_INFO(get_node()->get_logger(), "Halting the robot due to detected object.");
            break;
          }
          else if (car_movement == SLOW_DOWN)
          {
            halt_ = car_movement;
            RCLCPP_INFO(get_node()->get_logger(), "Slowing down the robot due to detected object.");
          }
          else
          {
            if (current_objects->object[i].label)
            {
              //RCLCPP_INFO(get_node()->get_logger(), "Person is the detected object.");
            }
          }
        }
        //RCLCPP_INFO(get_node()->get_logger(), "Object tracking count: %d", object_tracking_);
      }
    }


    if (!std::isnan(current_data->velocity) && !std::isnan(current_data->angle))
    {
      reference_velocity_ = current_data->velocity;
      reference_angle_ = current_data->angle;
      manual_override_ = current_data->manual;
      if (tracked_object_id_ % 1000 == 0) {
        RCLCPP_INFO(get_node()->get_logger(), "Received new reference velocity: %f", current_data->velocity);
        RCLCPP_INFO(get_node()->get_logger(), "Received new point x: %f", state_interfaces_[NEXT_POS_X].get_value());
        RCLCPP_INFO(get_node()->get_logger(), "Received new point y: %f", state_interfaces_[NEXT_POS_Y].get_value());
        RCLCPP_INFO(get_node()->get_logger(), "Received new point z: %f", state_interfaces_[NEXT_POS_Z].get_value());
      }
    }
    tracked_object_id_++;
    update_odometry(period);

    if (!std::isnan(reference_velocity_) && !std::isnan(reference_angle_))
    {
      const auto age_of_last_command = time - (*(input_ref_teleop_.readFromRT()))->header.stamp;
      //RCLCPP_INFO(get_node()->get_logger(), "Update 2: %f seconds", age_of_last_command.seconds());  
      const auto timeout =
      age_of_last_command > ref_timeout_ && ref_timeout_ != rclcpp::Duration::from_seconds(0);

      // store (for open loop odometry) and set commands
      last_velocity_ = timeout ? 0.0 : reference_velocity_;
      last_angle_ = timeout ? 0.0 : reference_angle_;
      
      if (halt_ == EMERGENCY_STOP)
      {
        auto [velocity_commands, steering_commands] = odometry_.get_commands(0.0);
        command_interfaces_[CMD_VELOCITY].set_value(velocity_commands);
        if (manual_override_ == 0) {
          command_interfaces_[CMD_STEERING].set_value(steering_commands);
        }
        //RCLCPP_INFO(get_node()->get_logger(), "Emergency stop activated. Setting velocity to 0.");
      }
      else if (halt_ == SLOW_DOWN)
      {
        auto [velocity_commands, steering_commands] = odometry_.get_commands(1.0);
        command_interfaces_[CMD_VELOCITY].set_value(velocity_commands);
        if (manual_override_ == 0) {
          command_interfaces_[CMD_STEERING].set_value(steering_commands);
        }
        //RCLCPP_INFO(get_node()->get_logger(), "Slow down activated. Setting velocity to 1.0.");
      }
      else
      {
        auto [velocity_commands, steering_commands] = odometry_.get_commands(last_velocity_);
        command_interfaces_[CMD_VELOCITY].set_value(velocity_commands);
        if (manual_override_ == 0) {
          command_interfaces_[CMD_STEERING].set_value(steering_commands);
        }
        else {
          command_interfaces_[CMD_STEERING].set_value(reference_angle_);
        }
        //RCLCPP_INFO(get_node()->get_logger(), "Setting velocity to %f and steering to %f.", last_velocity_, last_angle_);

      }
    }

    //utility::reset_controller_reference_teleopcommand_msg(*((input_ref_teleop_.readFromRT())), get_node());
    //utility::reset_controller_reference_image_msg(*(input_ref_img_.readFromRT()), get_node());

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

      //TO DO SOMETHING 
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

    reference_velocity_ = std::numeric_limits<double>::quiet_NaN();
    reference_angle_ = std::numeric_limits<double>::quiet_NaN();
    halt_ = RUNNING;

    return controller_interface::return_type::OK;
  }

  void RobotSteeringController::reference_callback(
    const std::shared_ptr<ImgAnalyzeMsg> msg)
  {
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Timestamp in header is missing, using current time as command timestamp.");
        msg->header.stamp = get_node()->now();
    }
    const auto age_of_last_command = get_node()->now() - msg->middle_header.stamp;
    //RCLCPP_INFO(get_node()->get_logger(), "Reference callback: %f seconds", age_of_last_command.seconds());

    if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_)
    {
      input_ref_img_.writeFromNonRT(msg);
    }
  }

  void RobotSteeringController::reference_teleop(
    const std::shared_ptr<TeleopCommand> msg)
  {
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Timestamp in header is missing, using current time as command timestamp.");
        msg->header.stamp = get_node()->now();
    }
    const auto age_of_last_command = get_node()->now() - msg->header.stamp;
    //RCLCPP_INFO(get_node()->get_logger(), "Reference teleop: %f seconds", age_of_last_command.seconds());

    if (ref_timeout_ == rclcpp::Duration::from_seconds(0) || age_of_last_command <= ref_timeout_)
    {
      input_ref_teleop_.writeFromNonRT(msg);
    }
  }

  int RobotSteeringController::object_detection(int label, double point3d_x, double point3d_y, double point3d_z)
  {

    const double corridor = (wheelbase_ * 0.75);
    bool in_corridor = (std::abs(point3d_x) < corridor);

    if (in_corridor)
    {
      if (point3d_z < stop_distance_)
        return EMERGENCY_STOP;
      else if (point3d_z < slow_distance_)
        return SLOW_DOWN;
    }
    return RUNNING;
  }

  void RobotSteeringController::halt()
  {
    auto logger = get_node()->get_logger();
    for (size_t i = 0; i < command_interfaces_.size(); i++)
    {
      command_interfaces_[i].set_value(0.0);
    }
  }

}  // namespace robot_steering_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_steering_controller::RobotSteeringController,
  controller_interface::ControllerInterface)
