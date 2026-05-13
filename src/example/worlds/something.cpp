#include "example/gps.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <thread>
#include <mutex>
#include <cmath>
#include <math.h>

// serial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// ethernet
#include <sys/socket.h>
#include <arpa/inet.h>

//////////////////////////////////////////////////////////////
// ANPP PARSER
//////////////////////////////////////////////////////////////

struct ANPPPacket
{
    uint8_t id;
    uint8_t length;
    std::vector<uint8_t> payload;
};

bool parse_anpp(const uint8_t* data, size_t size, ANPPPacket &pkt)
{
    if (size < 5) return false;

    pkt.id = data[1];
    pkt.length = data[2];

    if (size < pkt.length + 5) return false;

    pkt.payload.assign(data + 5, data + 5 + pkt.length);

    return true;
}

//////////////////////////////////////////////////////////////
// HARDWARE CLASS
//////////////////////////////////////////////////////////////

namespace example
{

GPSData decode_packet_20(const ANPPPacket &pkt)
{
    GPSData data;

    if (pkt.id != 20 || pkt.payload.size() < 100)
        return data;


    const uint8_t* pkg_data = pkt.payload.data();

    // f64 fields (8 bytes each)
    double lat_rad;
    double lon_rad;
    double height;
    std::memcpy(&lat_rad, pkg_data + 12, sizeof(double));
    std::memcpy(&lon_rad, pkg_data + 20, sizeof(double));
    std::memcpy(&height, pkg_data + 28, sizeof(double));

    // f32 fields (4 bytes each)
    float vel_north;
    float vel_east;
    float heading;
    std::memcpy(&vel_north, pkg_data + 36, sizeof(float));
    std::memcpy(&vel_east, pkg_data + 40, sizeof(float));
    std::memcpy(&heading, pkg_data + 72, sizeof(float));

    data.latitude  = lat_rad * 180.0 / M_PI;
    data.longitude = lon_rad * 180.0 / M_PI;
    data.altitude  = height;
    data.velocity_x = static_cast<double>(vel_north);
    data.velocity_y = static_cast<double>(vel_east);
    data.heading = static_cast<double>(heading);

    return data;
}

void GpsHardware::open_socket()
{
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    addr.sin_addr.s_addr = INADDR_ANY;

    bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr));
}

void GpsHardware::open_serial(const std::string &device)
{
    serial_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    struct termios tty{};
    tcgetattr(serial_fd_, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tcsetattr(serial_fd_, TCSANOW, &tty);
}

void GpsHardware::ethernet_loop()
{
    uint8_t buffer[1024];

    while (running_) {
        int n = recv(socket_fd_, buffer, sizeof(buffer), 0);

        if (n <= 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // --- Try ANPP ---
        ANPPPacket pkt;
        if (parse_anpp(buffer, n, pkt)) {
            if (pkt.id == 20) {
                GPSData data = decode_packet_20(pkt);
                std::lock_guard<std::mutex> lock(gps_mutex_);
                latest_data_ = data;
                continue;
            }
            else 
            {
                continue;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void GpsHardware::serial_loop()
{
    uint8_t buffer[512];

    while (running_) {
        int n = ::read(serial_fd_, buffer, sizeof(buffer));

        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // --- Try ANPP ---
        ANPPPacket pkt;
        if (parse_anpp(buffer, n, pkt)) {
            if (pkt.id == 20) {
                GPSData data = decode_packet_20(pkt);
                std::lock_guard<std::mutex> lock(gps_mutex_);
                latest_data_ = data;
                continue;
            }
            else 
                continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

//////////////////////////////////////////////////////////////
// ROS2 LIFECYCLE
//////////////////////////////////////////////////////////////

hardware_interface::CallbackReturn GpsHardware::on_init(
    const hardware_interface::HardwareInfo &info)
{
    if (SensorInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    joints_.clear();
    for (const auto &sensor : info_.sensors)
        joints_.push_back(Joint(sensor.name));

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GpsHardware::on_activate(
    const rclcpp_lifecycle::State &)
{
    running_ = true;

    if (interface_type_ == "serial") {
        open_serial("/dev/ttyUSB0");
        reader_thread_ = std::thread(&GpsHardware::serial_loop, this);
    } else {
        open_socket();
        reader_thread_ = std::thread(&GpsHardware::ethernet_loop, this);
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GpsHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    running_ = false;

    if (reader_thread_.joinable())
        reader_thread_.join();

    if (serial_fd_ > 0) close(serial_fd_);
    if (socket_fd_ > 0) close(socket_fd_);

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type GpsHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    GPSData copy;
    {
        std::lock_guard<std::mutex> lock(gps_mutex_);
        copy = latest_data_;
    }

    for (auto &joint : joints_) {
        if (joint.joint_name.find("gps_sensor") != std::string::npos) {
            joint.state.latitude  = copy.latitude;
            joint.state.longitude = copy.longitude;
            joint.state.altitude  = copy.altitude;
            joint.state.velocity_x  = copy.velocity_x;
            joint.state.velocity_y  = copy.velocity_y;
            joint.state.heading  = copy.heading;
        }
    }

    return hardware_interface::return_type::OK;
}

//////////////////////////////////////////////////////////////

std::vector<hardware_interface::StateInterface>
GpsHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto &joint : joints_) {
        state_interfaces.emplace_back(joint.joint_name, "latitude",  &joint.state.latitude);
        state_interfaces.emplace_back(joint.joint_name, "longitude", &joint.state.longitude);
        state_interfaces.emplace_back(joint.joint_name, "altitude",  &joint.state.altitude);
        state_interfaces.emplace_back(joint.joint_name, "velocity_x",  &joint.state.velocity_x);
        state_interfaces.emplace_back(joint.joint_name, "velocity_y",  &joint.state.velocity_y);
        state_interfaces.emplace_back(joint.joint_name, "heading",  &joint.state.heading);
    }

    return state_interfaces;
}

rclcpp::Clock GpsHardware::get_clock()
{
    return rclcpp::Clock(RCL_ROS_TIME);
}

rclcpp::Logger GpsHardware::get_logger()
{
    return rclcpp::get_logger("GpsHardware");
}

} // namespace example

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(example::GpsHardware, hardware_interface::SensorInterface)




















#pragma once

#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <serial/serial.h>
#include <unordered_map>
#include <string>

namespace example
{
    struct GPSData
    {
        double latitude{0.0};
        double longitude{0.0};
        double altitude{0.0};
        double velocity_x{0.0};
        double velocity_y{0.0};
        double heading{0.0};
    };
    
    struct Joint
    {
        explicit Joint(const std::string &name) : joint_name(name)
        {
            state = GPSData();
        }

        Joint() = default;

        std::string joint_name;
        GPSData state;
    };
    class GpsHardware : public hardware_interface::SensorInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(GpsHardware)

        void open_socket();

        void open_serial(const std::string &device);

        void ethernet_loop();

        void serial_loop();

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

        // Read/Write
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // Logger & Clock
        rclcpp::Clock get_clock();
        rclcpp::Logger get_logger();

        // State/Command access (map-based)
        double get_state(const std::string &name);
        void set_state(const std::string &name, double value);

    private:
        rclcpp::Clock clock_;
        serial::Serial serial_;

        // Maps for joint states and commands
        // OLD:
        // std::unordered_map<std::string, Joint> map_;

        // NEW:
        std::vector<Joint> joints_;

        GPSData latest_data_;
        std::mutex gps_mutex_;
        std::atomic<bool> running_{false};
        std::thread reader_thread_;

        // config
        std::string interface_type_ = "serial"; // or "ethernet"

        // serial
        int serial_fd_ = -1;

        // ethernet
        int socket_fd_ = -1;
        std::string ip_ = "192.168.1.10";
        int port_ = 9000;
    };

} // namespace example













//SIMULATION
#include "example/gps.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace example
{

    hardware_interface::CallbackReturn GpsHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SensorInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "Initializing GpsHardware");

        map_.clear();

        // Initialize internal maps for states
        for (const auto &joint : info_.joints)
        {
            RCLCPP_INFO(get_logger(), "Adding joint '%s' to the hardware interface.", joint.name.c_str());
            map_[joint.name] = Joint(joint.name);
        }
        for (const auto &sensor : info_.sensors)
        {
            RCLCPP_INFO(get_logger(), "Adding sensor '%s' to the hardware interface.", sensor.name.c_str());
            map_[sensor.name] = Joint(sensor.name);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn GpsHardware::on_configure(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring GpsHardware");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn GpsHardware::on_activate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating ...please wait... 1");

        for (auto &joint : map_)
        {

            if (joint.second.joint_name.find("gps_sensor") != std::string::npos)
            {
                joint.second.state.latitude = 0.0;
                joint.second.state.longitude = 0.0;
                joint.second.state.altitude = 0.0;
            }
        }

        RCLCPP_INFO(get_logger(), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn GpsHardware::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating GpsHardware");
        // optionally reset internal state here
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type GpsHardware::read(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Here you could read sensors and update state_map_
        RCLCPP_DEBUG(get_logger(), "Reading state from GpsHardware");

        for (auto &joint : map_)
        {
            if (joint.second.joint_name.find("gps_sensor") != std::string::npos)
            {
                joint.second.state.latitude += 0.1;
                joint.second.state.longitude += 0.1;
                joint.second.state.altitude += 0.1;
            }
        }

        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> GpsHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto &joint : map_)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.second.joint_name, "latitude", &joint.second.state.latitude));

            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.second.joint_name, "longitude", &joint.second.state.longitude));

            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.second.joint_name, "altitude", &joint.second.state.altitude));
        }

        RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());

        for (auto s : state_interfaces)
        {
            RCLCPP_INFO(get_logger(), "Exported state interface '%s'.", s.get_name().c_str());
        }

        return state_interfaces;
    }

    rclcpp::Clock GpsHardware::get_clock()
    {
        return rclcpp::Clock(RCL_ROS_TIME);
    }

    rclcpp::Logger GpsHardware::get_logger()
    {
        return rclcpp::get_logger("GpsHardware");
    }

    double GpsHardware::get_state(const std::string &name)
    {
        for (const auto &joint : map_)
        {
            if (joint.second.joint_name == name)
            {
                return joint.second.state.latitude; // Gps: return latitude
            }
        }
        return 0.0;
    }

    void GpsHardware::set_state(const std::string &name, double value)
    {
        for (auto &joint : map_)
        {
            if (joint.second.joint_name == name)
            {
                joint.second.state.latitude = value; // Gps: set latitude
            }
        }
    }

} // namespace example

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(example::GpsHardware, hardware_interface::SensorInterface)














#include "example/gps.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace example
{

    hardware_interface::CallbackReturn GpsHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SensorInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "Initializing GpsHardware");

        map_.clear();

        // Initialize internal maps for states
        for (const auto &joint : info_.joints)
        {
            RCLCPP_INFO(get_logger(), "Adding joint '%s' to the hardware interface.", joint.name.c_str());
            map_[joint.name] = Joint(joint.name);
        }
        for (const auto &sensor : info_.sensors)
        {
            RCLCPP_INFO(get_logger(), "Adding sensor '%s' to the hardware interface.", sensor.name.c_str());
            map_[sensor.name] = Joint(sensor.name);
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn GpsHardware::on_configure(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring GpsHardware");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn GpsHardware::on_activate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating ...please wait... 1");

        for (auto &joint : map_)
        {

            if (joint.second.joint_name.find("gps_sensor") != std::string::npos)
            {
                joint.second.state.latitude = 0.0;
                joint.second.state.longitude = 0.0;
                joint.second.state.altitude = 0.0;
            }
        }

        RCLCPP_INFO(get_logger(), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn GpsHardware::on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating GpsHardware");
        // optionally reset internal state here
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type GpsHardware::read(
        const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Here you could read sensors and update state_map_
        RCLCPP_DEBUG(get_logger(), "Reading state from GpsHardware");

        for (auto &joint : map_)
        {
            if (joint.second.joint_name.find("gps_sensor") != std::string::npos)
            {
                joint.second.state.latitude += 0.1;
                joint.second.state.longitude += 0.1;
                joint.second.state.altitude += 0.1;
            }
        }

        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> GpsHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto &joint : map_)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.second.joint_name, "latitude", &joint.second.state.latitude));

            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.second.joint_name, "longitude", &joint.second.state.longitude));

            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.second.joint_name, "altitude", &joint.second.state.altitude));
        }

        RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());

        for (auto s : state_interfaces)
        {
            RCLCPP_INFO(get_logger(), "Exported state interface '%s'.", s.get_name().c_str());
        }

        return state_interfaces;
    }

    rclcpp::Clock GpsHardware::get_clock()
    {
        return rclcpp::Clock(RCL_ROS_TIME);
    }

    rclcpp::Logger GpsHardware::get_logger()
    {
        return rclcpp::get_logger("GpsHardware");
    }

    double GpsHardware::get_state(const std::string &name)
    {
        for (const auto &joint : map_)
        {
            if (joint.second.joint_name == name)
            {
                return joint.second.state.latitude; // Gps: return latitude
            }
        }
        return 0.0;
    }

    void GpsHardware::set_state(const std::string &name, double value)
    {
        for (auto &joint : map_)
        {
            if (joint.second.joint_name == name)
            {
                joint.second.state.latitude = value; // Gps: set latitude
            }
        }
    }

} // namespace example

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(example::GpsHardware, hardware_interface::SensorInterface)
