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
        double temperature_front_left{0.0};
        double temperature_front_right{0.0};
        double temperature_front_timestamp{0.0};
        double temperature_rear_left{0.0};
        double temperature_rear_right{0.0};
        double temperature_rear_timestamp{0.0};
        double velocity_front_left{0.0};
        double velocity_front_right{0.0};
        double velocity_front_timestamp{0.0};
        double velocity_rear_left{0.0};
        double velocity_rear_right{0.0};
        double velocity_rear_timestamp{0.0};
        double throttle_front_left{0.0};
        double throttle_front_right{0.0};
        double throttle_front_timestamp{0.0};
        double throttle_rear_left{0.0};
        double throttle_rear_right{0.0};
        double throttle_rear_timestamp{0.0};
        double reverse_front_left{0.0};
        double reverse_front_right{0.0};
        double reverse_front_timestamp{0.0};
        double reverse_rear_left{0.0};
        double reverse_rear_right{0.0};
        double reverse_rear_timestamp{0.0};
        double brake_front_left{0.0};
        double brake_front_right{0.0};
        double brake_front_timestamp{0.0};
        double brake_rear_left{0.0};
        double brake_rear_right{0.0};
        double brake_rear_timestamp{0.0};
        double steering_angle{0.0};
        double steering_angle_timestamp{0.0};
        double steering_torque_high{0.0};
        double steering_torque_low{0.0};
        double steering_torque_timestamp{0.0};
        double battery_voltage{0.0};
        double battery_voltage_timestamp{0.0};
        double controller_status{0.0};
        double controller_status_timestamp{0.0};
        double velocity{0.0};
        double direction{0.0};
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

    struct ControlData {
        float direction{0.0}; // Original precision: 2 decimal places
        float velocity{0.0};  // Original precision: 3 decimal places
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
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

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
        static constexpr double factor = 2.4 / 29500.0;

        // CAN IDs
        // Devices IDs
        static constexpr unsigned int jetson_device =    0b0000;
        static constexpr unsigned int mci_front_device = 0b0010;
        static constexpr unsigned int mci_rear_device =  0b0011;

        // Common IDs
        static constexpr unsigned int Reset =  0b100'0000;
        static constexpr unsigned int Status = 0b100'0001;

        // Message IDs Jetson
        static constexpr unsigned int Control =          0b000'0000;
        static constexpr unsigned int WheelState =       0b000'0001;
        static constexpr unsigned int StartAS =          0b000'0011;
        static constexpr unsigned int StopAS =           0b000'0100;
        static constexpr unsigned int StartMoving =      0b000'0101;
        static constexpr unsigned int StopMoving =       0b000'0110;

        // Message IDs MCI
        static constexpr unsigned int Throttle =         0b000'0000;
        static constexpr unsigned int Temperature =      0b000'0001;
        static constexpr unsigned int Torque =           0b000'0010;
        static constexpr unsigned int PidGains =         0b000'0011;

        static constexpr unsigned int IDJetson(unsigned int messageID) {
            return (jetson_device << 8) | messageID;
        };
        static constexpr unsigned int IDMCIFront(unsigned int messageID) {
            return (mci_front_device << 8) | messageID;
        };
        static constexpr unsigned int IDMCIRear(unsigned int messageID) {
            return (mci_rear_device << 8) | messageID;
        };

        bool sendWheelState(bool reverse);
        void EncodeControl(const ControlData& data, uint8_t* buffer);
        bool sendFrame(uint32_t canId, const uint8_t *data, uint8_t dlc);
        void openCan();
        void canLoop();

        rclcpp::Clock clock_;
        serial::Serial serial_;
        std::vector<Joint> joints_;

        int can_socket_fd_;
        bool can_running_;
        std::string can_interface_{"can0"};
        int bytes_received{0};
        int bytes_sent{0};
        bool prev_reverse{false};

        std::thread can_thread_;
        std::mutex can_mutex_;
        std::mutex can_fd_mutex_;

        JointValue latest_can_data_; // protegido por can_mutex_

        int write_front {0};
        int write_rear {0};
        int write_direction {0};
};

} // namespace example
