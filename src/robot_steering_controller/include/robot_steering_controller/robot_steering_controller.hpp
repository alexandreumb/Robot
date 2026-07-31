#ifndef ROBOT_STEERING_CONTROLLER__ROBOT_STEERING_CONTROLLER_HPP_
  #define ROBOT_STEERING_CONTROLLER__ROBOT_STEERING_CONTROLLER_HPP_

  #include <memory>

  #include "robot_steering_controller/robot_steering_controller_parameters.hpp"
  #include "robot_steering_controller/visibility_control.h"
  #include "controller_interface/controller_interface.hpp"
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
  #include "msgs/msg/teleop_command.hpp"
  #include "msgs/msg/object_struct.hpp"
  
  #include "robot_steering_controller/mmio_gpio.hpp"

  #include <utility>
  #include <limits>
  #include <gpiod.h>

  namespace robot_steering_controller
  {
  enum RobotState
  {
    RUNNING,
    EMERGENCY_STOP,
    SLOW_DOWN,
  };

  enum StateInterfaces
  {
    //WHEELS
    FRONT_LEFT_TEMP,
    FRONT_RIGHT_TEMP,
    FRONT_TEMP_TIME,
    FRONT_LEFT_VEL,
    FRONT_RIGHT_VEL,
    FRONT_VEL_TIME,
    FRONT_LEFT_THROTTLE,
    FRONT_RIGHT_THROTTLE,
    FRONT_THROTTLE_TIME,
    FRONT_LEFT_REVERSE,
    FRONT_RIGHT_REVERSE,
    FRONT_REVERSE_TIME,
    FRONT_LEFT_BRAKE,
    FRONT_RIGHT_BRAKE,
    FRONT_BRAKE_TIME,
    REAR_LEFT_TEMP,
    REAR_RIGHT_TEMP,
    REAR_TEMP_TIME,
    REAR_LEFT_VEL,
    REAR_RIGHT_VEL,
    REAR_VEL_TIME,
    REAR_LEFT_THROTTLE,
    REAR_RIGHT_THROTTLE,
    REAR_THROTTLE_TIME,
    REAR_LEFT_REVERSE,
    REAR_RIGHT_REVERSE,
    REAR_REVERSE_TIME,
    REAR_LEFT_BRAKE,
    REAR_RIGHT_BRAKE,
    REAR_BRAKE_TIME,
    //STEERING
    STEERING_ANGLE,
    STEERING_ANGLE_TIME,
    STEERING_TORQUE_HIGH,
    STEERING_TORQUE_LOW,
    STEERING_TORQUE_TIME,
    //GPS
    LONGITUDE,
    LATITUDE,
    HEIGHT,
    VEL_NORTH,
    VEL_EAST,
    HEADING,
    NEXT_POS_X,
    NEXT_POS_Y,
    NEXT_POS_Z
  };
  
  enum CommandInterfaces
  {
    CMD_VELOCITY,
    CMD_STEERING
  };

  static constexpr size_t NR_STATE_ITFS = 44;
  static constexpr size_t NR_CMD_ITFS = 2;
  static constexpr size_t NR_REF_ITFS = 2;

  class RobotSteeringController : public controller_interface::ControllerInterface
  {
  public:
    RobotSteeringController();

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_init() override;
  
    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
    
    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
      
    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

    ROBOT_STEERING_CONTROLLER__VISIBILITY_PUBLIC controller_interface::return_type
    update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    using ControllerTwistReferenceMsg = geometry_msgs::msg::TwistStamped;
    using ControllerStateMsgOdom = nav_msgs::msg::Odometry;
    using ControllerStateMsgTf = tf2_msgs::msg::TFMessage;
    using SteeringControllerStateMsg = control_msgs::msg::SteeringControllerStatus;
    using ImgAnalyzeMsg = msgs::msg::ImgAnalyzeMsg;
    using TeleopCommand = msgs::msg::TeleopCommand;
    using ObjectStruct = msgs::msg::ObjectStruct;

  protected:  
    bool update_odometry(const rclcpp::Duration & period);

    realtime_tools::RealtimeBuffer<std::shared_ptr<ImgAnalyzeMsg>> input_ref_img_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<TeleopCommand>> input_ref_teleop_;

    rclcpp::Subscription<ImgAnalyzeMsg>::SharedPtr ref_subscriber_image_ = nullptr;
    rclcpp::Subscription<TeleopCommand>::SharedPtr ref_subscriber_teleopcommand_ = nullptr;
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
    
    double last_velocity_{0.0};
    double last_angle_{0.0};
    double reference_velocity_{std::numeric_limits<double>::quiet_NaN()};
    double reference_angle_{std::numeric_limits<double>::quiet_NaN()};

    std::shared_ptr<robot_steering_controller::ParamListener> robot_param_listener_;
    robot_steering_controller::Params robot_params_;

    std::vector<std::string> gps_sensor_names_;
    std::vector<std::string> axes_names_;
    std::vector<std::string> direction_names_;

    // name constants for state interfaces
    size_t nr_state_itfs_{0};
    // name constants for command interfaces
    size_t nr_cmd_itfs_{0};

  private:
    void halt();
    int object_detection(int label, double point3d_x, double point3d_y, double point3d_z);

    //MmioGpio gpio;
    int halt_{0};
    double stop_distance_{0.0};
    double slow_distance_{0.0};
    double wheelbase_{0.0};    
    int object_tracking_{0};
    int manual_override_{0}; // 0 = no manual override, 1 = manual override
    int car_movement{RUNNING};

    std::vector<float> execute_time;

    ROBOT_STEERING_CONTROLLER__VISIBILITY_LOCAL void reference_callback(
      const std::shared_ptr<ImgAnalyzeMsg> msg);    
    
    ROBOT_STEERING_CONTROLLER__VISIBILITY_LOCAL void reference_teleop(
      const std::shared_ptr<TeleopCommand> msg);
  };

  }  // namespace robot_steering_controller

  #endif  // ROBOT_STEERING_CONTROLLER__ROBOT_STEERING_CONTROLLER_HPP_
