#include "example/robot.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <thread>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <sys/ioctl.h>

namespace example
{

#define SEND_CAN_COMMANDS 0

void Robot4FarmersHardware::open_can()
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    can_socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_fd_ < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to create CAN socket");
        return;
    }

    strcpy(ifr.ifr_name, can_interface_.c_str());
    if (ioctl(can_socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to get CAN interface index for %s", can_interface_.c_str());
        close(can_socket_fd_);
        can_socket_fd_ = -1;
        return;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to bind CAN socket to interface %s", can_interface_.c_str());
        close(can_socket_fd_);
        can_socket_fd_ = -1;
        return;
    }
}

void Robot4FarmersHardware::can_loop()
{
    struct can_frame frame;
    while (can_running_) {
        bytes_received = ::read(can_socket_fd_, &frame, sizeof(struct can_frame));
        
        if (bytes_received < 0) {
            RCLCPP_ERROR(get_logger(), "Error reading from CAN socket");
            break;
        } else if (bytes_received < sizeof(struct can_frame)) {
            RCLCPP_ERROR(get_logger(), "Incomplete CAN frame received");
            continue;
        } else {
            // Process the CAN frame (frame.can_id, frame.data, frame.can_dlc)
            {
                std::lock_guard<std::mutex> lock(can_mutex_);
                switch (frame.can_id)
                {
                    case 0x100: // Front Throttle
                        latest_can_data_.throttle_front_left = (frame.data[4] << 8) | frame.data[5];
                        latest_can_data_.throttle_front_right = (frame.data[6] << 8) | frame.data[7];
                        latest_can_data_.throttle_front_timestamp = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8)  | frame.data[3];
                        break;

                    case 0x101: // Front temperature
                        latest_can_data_.temperature_front_left = (frame.data[4] << 8) | frame.data[5];
                        latest_can_data_.temperature_front_right = (frame.data[6] << 8) | frame.data[7];
                        latest_can_data_.temperature_front_timestamp = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3];
                        break;

                    case 0x200: // Rear throttle
                        latest_can_data_.throttle_rear_left = (frame.data[4] << 8) | frame.data[5];
                        latest_can_data_.throttle_rear_right = (frame.data[6] << 8) | frame.data[7];
                        latest_can_data_.throttle_rear_timestamp = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) |frame.data[3];
                        break;

                    case 0x201: // Rear temperature
                        latest_can_data_.temperature_rear_left = (frame.data[4] << 8) | frame.data[5];
                        latest_can_data_.temperature_rear_right = (frame.data[6] << 8) | frame.data[7];
                        latest_can_data_.temperature_rear_timestamp = (frame.data[0] << 24) | (frame.data[1] << 16) | (frame.data[2] << 8) | frame.data[3];
                        break;

                    case 0x300: // Steering
                        latest_can_data_.steering_angle = (frame.data[0] << 8) | frame.data[1];
                        latest_can_data_.steering_torque_high = (frame.data[2] << 8) | frame.data[3];
                        latest_can_data_.steering_torque_low = (frame.data[4] << 8) | frame.data[5];
                        break;

                    default:
                        // unknown CAN ID
                        break;
                }
            }
        }
    }
}

hardware_interface::CallbackReturn Robot4FarmersHardware::on_init(
    const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "Initializing Robot4FarmersHardware");
    joints_.clear();
    // CRITICAL: Reserve exact size and fill with emplace_back
    joints_.reserve(info_.joints.size());

    for (const auto &joint : info_.joints)
    {
        if (joint.name.empty())
        {
            RCLCPP_ERROR(get_logger(), "Joint name cannot be empty");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        joints_.emplace_back(Joint(joint.name));
    }
    
    RCLCPP_INFO(get_logger(), "Initialized %zu joints", joints_.size());
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Robot4FarmersHardware::on_configure(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Configuring Robot4FarmersHardware");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Robot4FarmersHardware::on_activate(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating Robot4FarmersHardware");
    open_can();
    if (can_socket_fd_ < 0) {
        RCLCPP_ERROR(get_logger(), "Cannot activate: CAN socket not connected");
        return CallbackReturn::ERROR;
    }

    can_running_ = true;
    can_thread_ = std::thread(&Robot4FarmersHardware::can_loop, this);

    for (auto &joint : joints_)
    {
        if (joint.joint_name.find("rear") != std::string::npos)
        {
            joint.state.velocity_rear_left = 0;
            joint.state.velocity_rear_right = 0;
            joint.state.velocity_rear_timestamp = 0;
            joint.state.temperature_rear_left = 0;
            joint.state.temperature_rear_right = 0;
            joint.state.temperature_rear_timestamp = 0;
            joint.state.throttle_rear_left = 0;
            joint.state.throttle_rear_right = 0;
            joint.state.throttle_rear_timestamp = 0;
            joint.state.brake_rear_left = 0;
            joint.state.brake_rear_right = 0;
            joint.state.brake_rear_timestamp = 0;
            joint.state.reverse_rear_left = 0;
            joint.state.reverse_rear_right = 0;
            joint.state.reverse_rear_timestamp = 0;

            joint.command.velocity = 0;      
        }
        else if (joint.joint_name.find("front") != std::string::npos)
        {
            joint.state.velocity_front_left = 0;
            joint.state.velocity_front_right = 0;
            joint.state.velocity_front_timestamp = 0;
            joint.state.temperature_front_left = 0;
            joint.state.temperature_front_right = 0;
            joint.state.temperature_front_timestamp = 0;
            joint.state.throttle_front_left = 0;
            joint.state.throttle_front_right = 0;
            joint.state.throttle_front_timestamp = 0;
            joint.state.brake_front_left = 0;
            joint.state.brake_front_right = 0;
            joint.state.brake_front_timestamp = 0;
            joint.state.reverse_front_left = 0;
            joint.state.reverse_front_right = 0;
            joint.state.reverse_front_timestamp = 0;

            joint.command.velocity = 0;
        }
        else if (joint.joint_name.find("direction") != std::string::npos)
        {
            joint.state.steering_angle = 0;
            joint.state.steering_angle_timestamp = 0;
            joint.state.steering_torque_high = 0;
            joint.state.steering_torque_low = 0;
            joint.state.steering_torque_timestamp = 0;

            joint.command.direction = 0;
        }
    }

    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Robot4FarmersHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating Robot4FarmersHardware");

    can_running_ = false;

    if (can_socket_fd_ >= 0) 
    {
        close(can_socket_fd_);
        can_socket_fd_ = -1;
    }

    if (can_thread_.joinable())
    {    
        can_thread_.join();
    }


    for (auto &joint : joints_)
    {
        if (joint.joint_name.find("rear") != std::string::npos)
        {
            joint.state.velocity_rear_left = 0;
            joint.state.velocity_rear_right = 0;
            joint.state.velocity_rear_timestamp = 0;
            joint.state.temperature_rear_left = 0;
            joint.state.temperature_rear_right = 0;
            joint.state.temperature_rear_timestamp = 0;
            joint.state.throttle_rear_left = 0;
            joint.state.throttle_rear_right = 0;
            joint.state.throttle_rear_timestamp = 0;
            joint.state.brake_rear_left = 0;
            joint.state.brake_rear_right = 0;
            joint.state.brake_rear_timestamp = 0;
            joint.state.reverse_rear_left = 0;
            joint.state.reverse_rear_right = 0;
            joint.state.reverse_rear_timestamp = 0;

            joint.command.velocity = 0;
        }
        else if (joint.joint_name.find("front") != std::string::npos)
        {
            joint.state.velocity_front_left = 0;
            joint.state.velocity_front_right = 0;
            joint.state.velocity_front_timestamp = 0;
            joint.state.temperature_front_left = 0;
            joint.state.temperature_front_right = 0;
            joint.state.temperature_front_timestamp = 0;
            joint.state.throttle_front_left = 0;
            joint.state.throttle_front_right = 0;
            joint.state.throttle_front_timestamp = 0;
            joint.state.brake_front_left = 0;
            joint.state.brake_front_right = 0;
            joint.state.brake_front_timestamp = 0;
            joint.state.reverse_front_left = 0;
            joint.state.reverse_front_right = 0;
            joint.state.reverse_front_timestamp = 0;

            joint.command.velocity = 0;
        }
        else if (joint.joint_name.find("direction") != std::string::npos)
        {
            joint.state.steering_angle = 0;
            joint.state.steering_angle_timestamp = 0;
            joint.state.steering_torque_high = 0;
            joint.state.steering_torque_low = 0;
            joint.state.steering_torque_timestamp = 0;

            joint.command.direction = 0;
        }
    }

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Robot4FarmersHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &period)
{
    {
        std::lock_guard<std::mutex> lock(can_mutex_);
        for (auto &joint : joints_)
        {
            if (joint.joint_name.find("rear") != std::string::npos)
            {
                joint.state.velocity_rear_left = latest_can_data_.velocity_rear_left;
                joint.state.velocity_rear_right = latest_can_data_.velocity_rear_right;
                joint.state.velocity_rear_timestamp = latest_can_data_.velocity_rear_timestamp;
                joint.state.temperature_rear_left = latest_can_data_.temperature_rear_left;
                joint.state.temperature_rear_right = latest_can_data_.temperature_rear_right;
                joint.state.temperature_rear_timestamp = latest_can_data_.temperature_rear_timestamp;
                joint.state.throttle_rear_left = latest_can_data_.throttle_rear_left;
                joint.state.throttle_rear_right = latest_can_data_.throttle_rear_right;
                joint.state.throttle_rear_timestamp = latest_can_data_.throttle_rear_timestamp;
            }
            else if (joint.joint_name.find("front") != std::string::npos)
            {                
                joint.state.velocity_front_left = latest_can_data_.velocity_front_left;
                joint.state.velocity_front_right = latest_can_data_.velocity_front_right;
                joint.state.velocity_front_timestamp = latest_can_data_.velocity_front_timestamp;
                joint.state.temperature_front_left = latest_can_data_.temperature_front_left;
                joint.state.temperature_front_right = latest_can_data_.temperature_front_right;
                joint.state.temperature_front_timestamp = latest_can_data_.temperature_front_timestamp;
                joint.state.throttle_front_left = latest_can_data_.throttle_front_left;
                joint.state.throttle_front_right = latest_can_data_.throttle_front_right;
                joint.state.throttle_front_timestamp = latest_can_data_.throttle_front_timestamp;
            }
            else if (joint.joint_name.find("direction") != std::string::npos)
            {
                joint.state.steering_angle = latest_can_data_.steering_angle;
                joint.state.steering_angle_timestamp = latest_can_data_.steering_angle_timestamp;
                joint.state.steering_torque_high = latest_can_data_.steering_torque_high;
                joint.state.steering_torque_low = latest_can_data_.steering_torque_low;
                joint.state.steering_torque_timestamp = latest_can_data_.steering_torque_timestamp;
            }
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Robot4FarmersHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    if (can_socket_fd_ < 0)
    {
        RCLCPP_ERROR(get_logger(), "CAN socket not connected");
        return hardware_interface::return_type::ERROR;
    }
    // In a real implementation, you would write commands to the CAN bus here.
    // For this example, we just log the commands that would be sent.
    for (const auto &joint : joints_)    {

#if SEND_CAN_COMMANDS
        stuct can_frame frame{};
        frame.dlc = 8; // Data length code (number of bytes in data)
#endif
        if (joint.joint_name.find("rear") != std::string::npos)
        {
#if SEND_CAN_COMMANDS
            frame.can_id = 0x200; // Example CAN ID for rear throttle
            int16_t velocity_command = static_cast<int16_t>(joint.command.velocity);

            frame.data[0] = (velocity_command >> 8) & 0xFF; // High byte
            frame.data[1] = velocity_command & 0xFF;        // Low byte

            ssize_t bytes_written = ::write(can_socket_fd_, &frame, sizeof(struct can_frame));

            if (bytes_written != sizeof(frame))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to send rear velocity CAN frame");
            }
            else
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Rear velocity command sent: %d",
                    velocity);
            }
#else
            RCLCPP_INFO(get_logger(), "Writing command for %s: velocity=%.2f", joint.joint_name.c_str(), joint.command.velocity);
#endif
        }
        else if (joint.joint_name.find("front") != std::string::npos)
        {
#if SEND_CAN_COMMANDS
            frame.can_id = 0x100; // Example CAN ID for front throttle
            int16_t velocity_command = static_cast<int16_t>(joint.command.velocity);

            frame.data[0] = (velocity_command >> 8) & 0xFF; // High byte
            frame.data[1] = velocity_command & 0xFF;        // Low byte

            ssize_t bytes_written = ::write(can_socket_fd_, &frame, sizeof(struct can_frame));

            if (bytes_written != sizeof(frame))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to send front velocity CAN frame");
            }
            else
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Front velocity command sent: %d",
                    velocity);
            }
#else
            RCLCPP_INFO(get_logger(), "Writing command for %s: velocity=%.2f", joint.joint_name.c_str(), joint.command.velocity);
#endif
        }
        else if (joint.joint_name.find("direction") != std::string::npos)
        {
#if SEND_CAN_COMMANDS
            //::write(can_socket_fd_, &joint.command.direction, sizeof(joint.command.direction));
            RCLCPP_INFO(get_logger(), "Writing command for %s: direction=%.2f", joint.joint_name.c_str(), joint.command.direction);
#else
            RCLCPP_INFO(get_logger(), "Writing command for %s: direction=%.2f", joint.joint_name.c_str(), joint.command.direction);
#endif
        }
    }
    return hardware_interface::return_type::OK;
}

// Helper functions
double Robot4FarmersHardware::get_state(const std::string &name)
{
    for (auto &joint : joints_)
    {
        if (joint.joint_name == name)
        {
            if (name.find("rear") != std::string::npos)
            {
                return joint.state.velocity_rear_left;
            }
            else
            {
                return joint.state.velocity_front_left;
            }
        }
    }
    RCLCPP_ERROR(get_logger(), "Joint '%s' not found in state", name.c_str());
    return 0.0;
}

void Robot4FarmersHardware::set_state(const std::string &name, double value)
{
    for (auto &joint : joints_)
    {
        if (joint.joint_name == name)
        {
            if (name.find("rear") != std::string::npos)
            {
                joint.state.velocity_rear_left = value;
            }
            else if (name.find("front") != std::string::npos)
            {
                joint.state.velocity_front_left = value;
            }
            return;
        }
    }
    RCLCPP_ERROR(get_logger(), "Joint '%s' not found in state", name.c_str());
}

double Robot4FarmersHardware::get_command(const std::string &name)
{
    for (auto &joint : joints_)
    {
        if (joint.joint_name == name)
        {
            if (name.find("rear") != std::string::npos)
            {
                return joint.command.velocity;
            }
            else if (name.find("front") != std::string::npos)
            {
                return joint.command.velocity; // Assuming front joints use velocity_front_left for command
            }
            else if (name.find("direction") != std::string::npos)
            {
                return joint.command.direction;
            }
        }
    }
    RCLCPP_ERROR(get_logger(), "Joint '%s' not found in command", name.c_str());
    return 0.0;
}

void Robot4FarmersHardware::set_command(const std::string &name, double value)
{
    for (auto &joint : joints_)
    {
        if (joint.joint_name == name)
        {
            if (name.find("rear") != std::string::npos)
            {
                joint.command.velocity = value;
            }
            else if (name.find("front") != std::string::npos)
            {
                joint.command.velocity = value;
            }
            else if (name.find("direction") != std::string::npos)
            {
                joint.command.direction = value;
            }
            return;
        }
    }
    RCLCPP_ERROR(get_logger(), "Joint '%s' not found in command", name.c_str());
}

rclcpp::Clock Robot4FarmersHardware::get_clock()
{
    return clock_;
}

rclcpp::Logger Robot4FarmersHardware::get_logger()
{
    return rclcpp::get_logger("Robot4FarmersHardware");
}

std::vector<hardware_interface::StateInterface> Robot4FarmersHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto &joint : joints_)
    {
        if (joint.joint_name.find("rear") != std::string::npos)
        {
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "velocity_rear_left", 
                &joint.state.velocity_rear_left));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "velocity_rear_right", 
                &joint.state.velocity_rear_right));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface( 
                joint.joint_name, 
                "velocity_rear_timestamp", 
                &joint.state.velocity_rear_timestamp));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "temperature_rear_left", 
                &joint.state.temperature_rear_left));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "temperature_rear_right", 
                &joint.state.temperature_rear_right));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "temperature_rear_timestamp", 
                &joint.state.temperature_rear_timestamp));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "throttle_rear_left", 
                &joint.state.throttle_rear_left));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "throttle_rear_right", 
                &joint.state.throttle_rear_right));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "throttle_rear_timestamp", 
                &joint.state.throttle_rear_timestamp));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "brake_rear_left", 
                &joint.state.brake_rear_left));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "brake_rear_right", 
                &joint.state.brake_rear_right));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "brake_rear_timestamp", 
                &joint.state.brake_rear_timestamp));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "reverse_rear_left", 
                &joint.state.reverse_rear_left));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "reverse_rear_right", 
                &joint.state.reverse_rear_right));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "reverse_rear_timestamp", 
                &joint.state.reverse_rear_timestamp));
        }
        else if (joint.joint_name.find("front") != std::string::npos)
        {
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "velocity_front_left", 
                &joint.state.velocity_front_left));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "velocity_front_right", 
                &joint.state.velocity_front_right));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "velocity_front_timestamp", 
                &joint.state.velocity_front_timestamp));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "temperature_front_left", 
                &joint.state.temperature_front_left));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "temperature_front_right", 
                &joint.state.temperature_front_right));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "temperature_front_timestamp", 
                &joint.state.temperature_front_timestamp));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "throttle_front_left", 
                &joint.state.throttle_front_left));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "throttle_front_right", 
                &joint.state.throttle_front_right));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "throttle_front_timestamp", 
                &joint.state.throttle_front_timestamp));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "brake_front_left", 
                &joint.state.brake_front_left));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(    
                joint.joint_name, 
                "brake_front_right", 
                &joint.state.brake_front_right));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "brake_front_timestamp", 
                &joint.state.brake_front_timestamp));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "reverse_front_left", 
                &joint.state.reverse_front_left));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "reverse_front_right", 
                &joint.state.reverse_front_right));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "reverse_front_timestamp", 
                &joint.state.reverse_front_timestamp));
        }
        else if (joint.joint_name.find("direction") != std::string::npos)
        {
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "steering_angle", 
                &joint.state.steering_angle));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "steering_angle_timestamp", 
                &joint.state.steering_angle_timestamp));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "steering_torque_high", 
                &joint.state.steering_torque_high));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "steering_torque_low", 
                &joint.state.steering_torque_low));
            state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.joint_name, 
                "steering_torque_timestamp", 
                &joint.state.steering_torque_timestamp));
        }
    }

    RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());
    return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> Robot4FarmersHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Use index-based loop instead of range-based
    for (auto &joint : joints_)
    {
        if (joint.joint_name.find("rear") != std::string::npos)
        {
            command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                joint.joint_name, 
                hardware_interface::HW_IF_VELOCITY, 
                &joint.command.velocity));
        }
        else if (joint.joint_name.find("front") != std::string::npos)
        {
            command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                joint.joint_name, 
                hardware_interface::HW_IF_VELOCITY, 
                &joint.command.velocity));
        }
        else if (joint.joint_name.find("direction") != std::string::npos)
        {
            command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                joint.joint_name, 
                "direction", 
                &joint.command.direction));
        }
    }

    RCLCPP_INFO(get_logger(), "Exported %zu command interfaces.", command_interfaces.size());
    return command_interfaces;
}

} // namespace example

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(example::Robot4FarmersHardware, hardware_interface::SystemInterface)