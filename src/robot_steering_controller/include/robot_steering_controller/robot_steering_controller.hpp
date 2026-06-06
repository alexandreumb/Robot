#ifndef ROBOT_STEERING_CONTROLLER__ROBOT_STEERING_CONTROLLER_HPP_
  #define ROBOT_STEERING_CONTROLLER__ROBOT_STEERING_CONTROLLER_HPP_

  #include <memory>

  #include "robot_steering_controller/robot_steering_controller_parameters.hpp"
  #include "robot_steering_controller/visibility_control.h"
  #include "controller_interface/chainable_controller_interface.hpp"
  #include "hardware_interface/handle.hpp"
  #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
  #include "rclcpp_lifecycle/state.hpp"
  #include "realtime_tools/realtime_buffer.hpp"
  #include "realtime_tools/realtime_publisher.hpp"
  #include "std_srvs/srv/set_bool.hpp"
  #include "robot_steering_controller/steering_odometry.hpp"

  #include "control_msgs/msg/steering_controller_status.hpp"
  #include "geometry_msgs/msg/twist_stamped.hpp"
  #include "nav_msgs/msg/odometry.hpp"
  #include "tf2_msgs/msg/tf_message.hpp"
  #include "msgs/msg/img_analyze_msg.hpp"
  #include "msgs/msg/velocity.hpp"
  #include "msgs/msg/object_struct.hpp"

  #include <utility>

  namespace robot_steering_controller
  {
  // name constants for state interfaces
  static constexpr size_t POS_Z = 35;
  static constexpr size_t POS_Y = 36;
  static constexpr size_t POS_X = 37;
  static constexpr size_t VEL_NORTH = 38;
  static constexpr size_t VEL_EAST = 39;
  static constexpr size_t HEADING = 40;

  // name constants for command interfaces
  static constexpr size_t CMD_TRACTION_WHEELS = 0;
  static constexpr size_t CMD_STEERING = 1;

  static constexpr size_t NR_STATE_ITFS = 3;
  static constexpr size_t NR_CMD_ITFS = 2;
  static constexpr size_t NR_REF_ITFS = 2;

  class RobotSteeringController : public controller_interface::ChainableControllerInterface
  {
  public:
    RobotSteeringController();

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_init() override;
    
    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC bool update_odometry(
    const rclcpp::Duration & period);
  
    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
    
    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
      
    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::return_type
    update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::return_type
    update_reference_from_subscribers() override;

    std::vector<std::string> rear_wheels_state_names_;
    std::vector<std::string> front_wheels_state_names_;

    using ControllerTwistReferenceMsg = geometry_msgs::msg::TwistStamped;
    using ControllerStateMsgOdom = nav_msgs::msg::Odometry;
    using ControllerStateMsgTf = tf2_msgs::msg::TFMessage;
    using SteeringControllerStateMsg = control_msgs::msg::SteeringControllerStatus;
    using ImgAnalyzeMsg = msgs::msg::ImgAnalyzeMsg;
    using Velocity = msgs::msg::Velocity;
    using ObjectStruct = msgs::msg::ObjectStruct;

  protected:
    controller_interface::CallbackReturn set_interface_numbers(size_t nr_state_itfs, size_t nr_cmd_itfs, size_t nr_ref_itfs);
  
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    realtime_tools::RealtimeBuffer<std::shared_ptr<ImgAnalyzeMsg>> input_ref_img_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<Velocity>> input_ref_vel_;

    rclcpp::Subscription<ImgAnalyzeMsg>::SharedPtr ref_subscriber_image_ = nullptr;
    rclcpp::Subscription<Velocity>::SharedPtr ref_subscriber_velocity_ = nullptr;
    rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);  // 0ms

    using ControllerStatePublisherOdom = realtime_tools::RealtimePublisher<ControllerStateMsgOdom>;
    using ControllerStatePublisherTf = realtime_tools::RealtimePublisher<ControllerStateMsgTf>;
    using ControllerStatePublisher = realtime_tools::RealtimePublisher<SteeringControllerStateMsg>;
    
    rclcpp::Publisher<ControllerStateMsgOdom>::SharedPtr odom_s_publisher_;
    rclcpp::Publisher<ControllerStateMsgTf>::SharedPtr tf_odom_s_publisher_;
    rclcpp::Publisher<SteeringControllerStateMsg>::SharedPtr controller_s_publisher_;
    
    std::unique_ptr<ControllerStatePublisherOdom> rt_odom_state_publisher_;
    std::unique_ptr<ControllerStatePublisherTf> rt_tf_odom_state_publisher_;
    std::unique_ptr<ControllerStatePublisher> controller_state_publisher_;
    
    steering_odometry::SteeringOdometry odometry_;
    
    double last_linear_velocity_{ 0.0 };
    double last_angular_velocity_{ 0.0 };

    std::shared_ptr<robot_steering_controller::ParamListener> robot_param_listener_;
    robot_steering_controller::Params robot_params_;

    std::vector<std::string> gps_sensor_names_;
    std::vector<std::string> axes_names_;
    std::vector<std::string> direction_names_;

    int id{0};

    // name constants for state interfaces
    size_t nr_state_itfs_{3};
    // name constants for command interfaces
    size_t nr_cmd_itfs_{2};
    // name constants for reference interfaces
    size_t nr_ref_itfs_{3};

  private:
    ROBOT_STEERING_CONTROLLER__VISIBILITY_LOCAL void reference_callback(
      const std::shared_ptr<ImgAnalyzeMsg> msg);    
    
    ROBOT_STEERING_CONTROLLER__VISIBILITY_LOCAL void reference_velocity(
      const std::shared_ptr<Velocity> msg);
  };

  
  }  // namespace robot_steering_controller

  #endif  // ROBOT_STEERING_CONTROLLER__ROBOT_STEERING_CONTROLLER_HPP_
