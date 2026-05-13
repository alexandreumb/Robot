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

  #ifndef ROBOT_STEERING_CONTROLLER__ROBOT_STEERING_CONTROLLER_HPP_
  #define ROBOT_STEERING_CONTROLLER__ROBOT_STEERING_CONTROLLER_HPP_

  #include <memory>

  #include "robot_steering_controller/robot_steering_controller_parameters.hpp"
  #include "robot_steering_controller/visibility_control.h"
  #include "steering_controllers_library/steering_controllers_library.hpp"

  namespace robot_steering_controller
  {
  // name constants for state interfaces
  static constexpr size_t STATE_TRACTION_RIGHT_WHEEL = 0;
  static constexpr size_t STATE_TRACTION_LEFT_WHEEL = 1;
  static constexpr size_t STATE_STEER_RIGHT_WHEEL = 2;
  static constexpr size_t STATE_STEER_LEFT_WHEEL = 3;

  // name constants for command interfaces
  static constexpr size_t CMD_TRACTION_RIGHT_WHEEL = 0;
  static constexpr size_t CMD_TRACTION_LEFT_WHEEL = 1;
  static constexpr size_t CMD_STEER_RIGHT_WHEEL = 2;
  static constexpr size_t CMD_STEER_LEFT_WHEEL = 3;

  static constexpr size_t NR_STATE_ITFS = 4;
  static constexpr size_t NR_CMD_ITFS = 4;
  static constexpr size_t NR_REF_ITFS = 2;

  class RobotSteeringController : public steering_controllers_library::SteeringControllersLibrary
  {
  public:
    RobotSteeringController();

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn
    configure_odometry() override;

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC bool update_odometry(
      const rclcpp::Duration & period) override;

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC void
    initialize_implementation_parameter_listener() override;

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

  protected:
    std::shared_ptr<robot_steering_controller::ParamListener> robot_param_listener_;
    robot_steering_controller::Params robot_params_;
    std::vector<std::string> gps_sensor_names_;
  };
  }  // namespace robot_steering_controller

  #endif  // ROBOT_STEERING_CONTROLLER__ROBOT_STEERING_CONTROLLER_HPP_
