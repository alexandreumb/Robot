#include "example/gps.hpp"

// SDK — incluído aqui e não no header
#include "gnss_compass_utils/read_gnss_compass.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace example
{

#define GPS_ACTIVE 1

//////////////////////////////////////////////////////////////
// SOCKET
//////////////////////////////////////////////////////////////

void GpsHardware::open_socket()
{
    struct addrinfo hints{}, *addr;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;       // TCP
    hints.ai_protocol = IPPROTO_TCP;

    if (getaddrinfo(ip_.c_str(), std::to_string(port_).c_str(), &hints, &addr) != 0) {
        RCLCPP_ERROR(get_logger(), "Failed to resolve address %s:%d", ip_.c_str(), port_);
        return;
    }

    socket_fd_ = socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to create socket");
        freeaddrinfo(addr);
        return;
    }

    if (connect(socket_fd_, addr->ai_addr, addr->ai_addrlen) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to connect to %s:%d", ip_.c_str(), port_);
        close(socket_fd_);
        socket_fd_ = -1;
        freeaddrinfo(addr);
        return;
    }
    freeaddrinfo(addr);
    RCLCPP_INFO(get_logger(), "Socket connected to %s:%d", ip_.c_str(), port_);
}

//////////////////////////////////////////////////////////////
// ETHERNET LOOP (thread separada)
//////////////////////////////////////////////////////////////

void GpsHardware::ethernet_loop()
{
    GPSData buffer{};
    
    RCLCPP_INFO(get_logger(), "Ethernet loop");

    while (running_) {

        // ── 1. Receber dados para o buffer do decoder ──────────
        bytes_received = ::read(socket_fd_, an_decoder_pointer(&an_decoder_), an_decoder_size(&an_decoder_));
                
        if (bytes_received <= 0) {
            RCLCPP_INFO(get_logger(), "No bytes received");
            // socket fechado ou erro — aguarda e tenta continuar
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        an_decoder_increment(&an_decoder_, bytes_received);

        // ── 2. Decodificar todos os pacotes disponíveis ─────────
        while ((an_packet_ = an_packet_decode(&an_decoder_)) != nullptr)
        {
            if (an_packet_->id == packet_id_system_state)
            {
                if (decode_system_state_packet(&system_state_packet_, an_packet_) == 0)
                {
                    // Log apenas quando o tipo de fix muda
                    int fix_type = static_cast<int>(
                        system_state_packet_.filter_status.b.gnss_fix_type);

                    if (fix_type != gnss_fix_type_prev_) {
                        gnss_fix_type_prev_ = fix_type;
                        switch (fix_type) {
                            case 0: RCLCPP_INFO(get_logger(), "GNSS fix lost");                   break;
                            case 1: RCLCPP_INFO(get_logger(), "GNSS fix: 2D");                    break;
                            case 2: RCLCPP_INFO(get_logger(), "GNSS fix: 3D");                    break;
                            case 3: RCLCPP_INFO(get_logger(), "GNSS fix: SBAS");                  break;
                            case 4: RCLCPP_INFO(get_logger(), "GNSS fix: Differential");          break;
                            case 5: RCLCPP_INFO(get_logger(), "GNSS fix: OmniSTAR/StarFire");     break;
                            case 6: RCLCPP_INFO(get_logger(), "GNSS fix: RTK Float");             break;
                            case 7: RCLCPP_INFO(get_logger(), "GNSS fix: RTK Fixed");             break;
                            default: RCLCPP_INFO(get_logger(), "GNSS fix: Unknown (%d)", fix_type); break;
                        }
                    }

                    // Preencher buffer — nomes consistentes com GPSData
                    buffer.gnss_fix = static_cast<double>(
                        system_state_packet_.filter_status.b.gnss_fix_type);
                    buffer.counter++;

                    buffer.latitude  = system_state_packet_.latitude;
                    buffer.longitude = system_state_packet_.longitude;
                    buffer.height    = system_state_packet_.height;

                    buffer.velocity_north = system_state_packet_.velocity[0];
                    buffer.velocity_east  = system_state_packet_.velocity[1];
                    buffer.velocity_down  = system_state_packet_.velocity[2];

                    buffer.body_acceleration_x = system_state_packet_.body_acceleration[0];
                    buffer.body_acceleration_y = system_state_packet_.body_acceleration[1];
                    buffer.body_acceleration_z = system_state_packet_.body_acceleration[2];

                    buffer.g_force = system_state_packet_.g_force;

                    buffer.roll    = system_state_packet_.orientation[0];
                    buffer.pitch   = system_state_packet_.orientation[1];
                    buffer.heading = system_state_packet_.orientation[2];

                    buffer.angular_velocity_x = system_state_packet_.angular_velocity[0];
                    buffer.angular_velocity_y = system_state_packet_.angular_velocity[1];
                    buffer.angular_velocity_z = system_state_packet_.angular_velocity[2];

                    buffer.standard_deviation_latitude  = system_state_packet_.standard_deviation[0];
                    buffer.standard_deviation_longitude = system_state_packet_.standard_deviation[1];
                    buffer.standard_deviation_height    = system_state_packet_.standard_deviation[2];
                }
            }
            else if (an_packet_->id == packet_id_raw_sensors)
            {
                if (decode_raw_sensors_packet(&raw_sensors_packet_, an_packet_) == 0)
                {
                    buffer.accelerometer_x = raw_sensors_packet_.accelerometers[0];
                    buffer.accelerometer_y = raw_sensors_packet_.accelerometers[1];
                    buffer.accelerometer_z = raw_sensors_packet_.accelerometers[2];
                }
            }
            else
            {
                RCLCPP_DEBUG(get_logger(), "Received unknown packet id: %d", an_packet_->id);
            }

            an_packet_free(&an_packet_);
        }

        // ── 3. Publicar buffer protegido por mutex ──────────────
        {
            std::lock_guard<std::mutex> lock(gps_mutex_);
            latest_data_ = buffer;
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

    // Lê parâmetros do URDF/ros2_control se existirem
    if (info_.hardware_parameters.count("ip"))
        ip_ = info_.hardware_parameters.at("ip");
    if (info_.hardware_parameters.count("port"))
        port_ = std::stoi(info_.hardware_parameters.at("port"));

    joints_.clear();
    for (const auto &sensor : info_.sensors)
        joints_.emplace_back(sensor.name);

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GpsHardware::on_configure(
    const rclcpp_lifecycle::State &)
{
#if GPS_ACTIVE
    an_decoder_initialise(&an_decoder_);
    gnss_fix_type_prev_ = -1;
    latest_data_        = GPSData{};
#endif
    RCLCPP_INFO(get_logger(), "GpsHardware configured (ip=%s port=%d)", ip_.c_str(), port_);
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GpsHardware::on_activate(
    const rclcpp_lifecycle::State &)
{
#if GPS_ACTIVE
    open_socket();
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(get_logger(), "Cannot activate: socket not connected");
        return CallbackReturn::ERROR;
    }

    running_ = true;
    reader_thread_ = std::thread(&GpsHardware::ethernet_loop, this);

#endif
    RCLCPP_INFO(get_logger(), "GpsHardware activated");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GpsHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
#if GPS_ACTIVE
    running_ = false;

    if (reader_thread_.joinable())
        reader_thread_.join();

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
#endif
    RCLCPP_INFO(get_logger(), "GpsHardware deactivated");
    return CallbackReturn::SUCCESS;
}

//////////////////////////////////////////////////////////////
// STATE INTERFACES
//////////////////////////////////////////////////////////////

std::vector<hardware_interface::StateInterface>
GpsHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto &joint : joints_) {
        const auto &n = joint.joint_name;
        state_interfaces.emplace_back(n, "gnss_fix",        &joint.state.gnss_fix        );
        state_interfaces.emplace_back(n, "latitude",        &joint.state.latitude        );
        state_interfaces.emplace_back(n, "longitude",       &joint.state.longitude       );
        state_interfaces.emplace_back(n, "height",          &joint.state.height          );
        state_interfaces.emplace_back(n, "velocity_north",  &joint.state.velocity_north  );
        state_interfaces.emplace_back(n, "velocity_east",   &joint.state.velocity_east   );
        state_interfaces.emplace_back(n, "velocity_down",   &joint.state.velocity_down   );
        state_interfaces.emplace_back(n, "roll",            &joint.state.roll            );
        state_interfaces.emplace_back(n, "pitch",           &joint.state.pitch           );
        state_interfaces.emplace_back(n, "heading",         &joint.state.heading         );
        state_interfaces.emplace_back(n, "angular_velocity_x", &joint.state.angular_velocity_x);
        state_interfaces.emplace_back(n, "angular_velocity_y", &joint.state.angular_velocity_y);
        state_interfaces.emplace_back(n, "angular_velocity_z", &joint.state.angular_velocity_z);
        state_interfaces.emplace_back(n, "accelerometer_x", &joint.state.accelerometer_x );
        state_interfaces.emplace_back(n, "accelerometer_y", &joint.state.accelerometer_y );
        state_interfaces.emplace_back(n, "accelerometer_z", &joint.state.accelerometer_z );
    }

    return state_interfaces;
}

//////////////////////////////////////////////////////////////
// READ — chamado pelo controller manager
//////////////////////////////////////////////////////////////

hardware_interface::return_type GpsHardware::read(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    GPSData copy;

#if GPS_ACTIVE
    {
        std::lock_guard<std::mutex> lock(gps_mutex_);
        copy = latest_data_;
    }
#endif

    for (auto &joint : joints_) {
        joint.state = copy;
        if (i % 100 == 0)
        {
            RCLCPP_INFO(get_logger(), "Writing command for %s: gnss_fix=%.2f", joint.joint_name.c_str(), joint.state.gnss_fix);
            RCLCPP_INFO(get_logger(), "Writing command for %s: acc=%.2f", joint.joint_name.c_str(), joint.state.accelerometer_y);
            RCLCPP_INFO(get_logger(), "Writing command for %s: rollx=%.2f", joint.joint_name.c_str(), joint.state.roll);
        }
    }

    i++;

    return hardware_interface::return_type::OK;
}

//////////////////////////////////////////////////////////////
// HELPERS
//////////////////////////////////////////////////////////////

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
