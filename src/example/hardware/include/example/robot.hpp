#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <serial/serial.h>
#include <unordered_map>
#include <string>
#include <thread>

namespace example
{
    struct JointValue
    {
        double temperature_front_left;
        double temperature_front_right;
        double temperature_front_timestamp;
        double temperature_rear_left;
        double temperature_rear_right;
        double temperature_rear_timestamp;
        double velocity_front_left;
        double velocity_front_right;
        double velocity_front_timestamp;
        double velocity_rear_left;
        double velocity_rear_right;
        double velocity_rear_timestamp;
        double throttle_front_left;
        double throttle_front_right;
        double throttle_front_timestamp;
        double throttle_rear_left;
        double throttle_rear_right;
        double throttle_rear_timestamp;
        double reverse_front_left;
        double reverse_front_right;
        double reverse_front_timestamp;
        double reverse_rear_left;
        double reverse_rear_right;
        double reverse_rear_timestamp;
        double brake_front_left;
        double brake_front_right;
        double brake_front_timestamp;
        double brake_rear_left;
        double brake_rear_right;
        double brake_rear_timestamp;
        double steering_angle;
        double steering_angle_timestamp;
        double steering_torque_high;
        double steering_torque_low;
        double steering_torque_timestamp;
        double battery_voltage;
        double battery_voltage_timestamp;
        double controller_status;
        double controller_status_timestamp;
        double velocity;
        double direction;
    };

    struct Joint
    {
        explicit Joint(const std::string &name) : joint_name(name)
        {
            state = JointValue();
            command = JointValue();
        }

        Joint() = default;

        std::string joint_name;
        JointValue state;
        JointValue command;
    };
    class Robot4FarmersHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Robot4FarmersHardware)

        // Lifecycle hooks
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override
        {
            RCLCPP_INFO(get_logger(), "Cleaning up Robot4FarmersHardware");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override
        {
            RCLCPP_INFO(get_logger(), "Shutting down Robot4FarmersHardware");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // Read/Write
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // Logger & Clock
        rclcpp::Clock get_clock();
        rclcpp::Logger get_logger();

        // State/Command access (map-based)
        double get_state(const std::string &name);
        void set_state(const std::string &name, double value);

        double get_command(const std::string &name);
        void set_command(const std::string &name, double value);

    private:
        void open_can();
        void can_loop();

        rclcpp::Clock clock_;
        serial::Serial serial_;
        
        // Maps for joint states and commands
        // OLD:
        // std::unordered_map<std::string, Joint> map_;

        // NEW:
        std::vector<Joint> joints_;

        int can_socket_fd_;
        bool can_running_;
        std::string can_interface_{"can0"};
        int bytes_received{0};


        std::thread can_thread_;
        std::mutex can_mutex_;

        JointValue latest_can_data_; // protegido por can_mutex_
    };

} // namespace example
