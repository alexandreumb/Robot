#include "example/gps.hpp"
#include "gnss_compass_utils/read_gnss_compass.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

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
#include <filesystem>

namespace example
{

#define GPS_ACTIVE 0
#define WAYPOINT_MIN_DIST_SQ 0.25

//////////////////////////////////////////////////////////////
// SOCKET
//////////////////////////////////////////////////////////////

void GpsHardware::openSocket()
{
    struct addrinfo hints{}, *addr;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    if (!read_)
    {
        const char* home = getenv("HOME");
        if (home == nullptr) {
            RCLCPP_ERROR(get_logger(), "HOME environment variable not set");
            return;
        }

        const std::string output_dir = std::string(home) + "/anpp_files";
        std::filesystem::create_directories(output_dir); 

        char filename[64];
        time_t rawtime = time(NULL);
        struct tm* timeinfo = localtime(&rawtime);
        snprintf(filename, sizeof(filename), "ANPP_waypoints_%02d-%02d-%02d-%02d-%02d-%02d.anpp", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

        std::string filepath = std::string(output_dir) + "/" + filename;
        log_file_ = fopen(filepath.c_str(), "wb");
        
        if (log_file_ == nullptr) 
        {
            RCLCPP_ERROR(get_logger(), "Failed to open log file: %s", filepath.c_str());
            return;
        } 
        else 
        {
            RCLCPP_INFO(get_logger(), "Logging waypoints to: %s", filepath.c_str());
        }
    }
    

    if (getaddrinfo(ip_.c_str(), std::to_string(port_).c_str(), &hints, &addr) != 0) 
    {
        RCLCPP_ERROR(get_logger(), "Failed to resolve address %s:%d", ip_.c_str(), port_);

        if (log_file_ != nullptr)
        {
            fflush(log_file_);
            fclose(log_file_);
        }
        return;
    }

    socket_fd_ = socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
    if (socket_fd_ < 0) 
    {
        RCLCPP_ERROR(get_logger(), "Failed to create socket");
        freeaddrinfo(addr);
        if (log_file_ != nullptr)
        {
            fflush(log_file_);
            fclose(log_file_);
        }
        return;
    }

    if (connect(socket_fd_, addr->ai_addr, addr->ai_addrlen) < 0) {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to connect to %s:%d : %s",
            ip_.c_str(),
            port_,
            strerror(errno));
        close(socket_fd_);
        socket_fd_ = -1;
        if (log_file_ != nullptr)
        {
            fflush(log_file_);
            fclose(log_file_);
        }
        freeaddrinfo(addr);
        return;
    }

    freeaddrinfo(addr);
    RCLCPP_INFO(get_logger(), "Socket connected to %s:%d", ip_.c_str(), port_);

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

//////////////////////////////////////////////////////////////
// ETHERNET LOOP (thread separada)
//////////////////////////////////////////////////////////////

void GpsHardware::ethernetLoop()
{
    GPSData buffer{};
    
    RCLCPP_INFO(get_logger(), "Ethernet loop");

    while (running_) 
    {

        // ── 1. Receber dados para o buffer do decoder ──────────
        bytes_received = ::read(socket_fd_, an_decoder_pointer(&an_decoder_), an_decoder_size(&an_decoder_));
                
        if (bytes_received <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            } else if (bytes_received == 0) {
                RCLCPP_ERROR(get_logger(), "Socket closed by remote host");
                break;
            } else {
                RCLCPP_ERROR(get_logger(), "Socket read error: %s", strerror(errno));
                break;
            }
        }

        if (!read_)
        {
            if (log_file_ != nullptr)
            {
                fwrite(an_decoder_pointer(&an_decoder_), sizeof(uint8_t), bytes_received, log_file_);
                if (flush_counter_++ == 100)
                {
                    fflush(log_file_);
                    flush_counter_ = 0;
                }
            }
            flush_counter_++;
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

                    if (fix_type != gnss_fix_type_prev_) 
                    {
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

    RCLCPP_ERROR(get_logger(), "Ethernet loop exiting — GPS data stream has stopped!");
    running_ = false;
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
    if (info_.hardware_parameters.count("read"))
    read_ = std::stoi(info_.hardware_parameters.at("read"));
        
    if (read_)
    {
        if (info_.hardware_parameters.count("file_name"))
            file_name_to_reproduce_ = info_.hardware_parameters.at("file_name");
    }     

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
    if (read_)
    {
        // Load entire file into waypoints vector upfront
        log_file_ = fopen(file_name_to_reproduce_.c_str(), "rb");
        if (log_file_ == nullptr) {
            RCLCPP_ERROR(get_logger(), "Cannot open waypoint file: %s", file_name_to_reproduce_.c_str());
            return CallbackReturn::ERROR;
        }

        // Your saved format is raw ANPP bytes, so you need to decode the
        // whole file first, then extract waypoints from decoded packets.
        // Read everything into a buffer:
        fseek(log_file_, 0, SEEK_END);
        long file_size = ftell(log_file_);
        rewind(log_file_);

        std::vector<uint8_t> raw(file_size);
        fread(raw.data(), sizeof(uint8_t), file_size, log_file_);
        fclose(log_file_);

        // Feed into the ANPP decoder and extract all system_state packets
        an_decoder_t decoder;
        an_decoder_initialise(&decoder);
        an_packet_t* packet;

        size_t offset = 0;
        while (offset < raw.size()) {
            size_t space = an_decoder_size(&decoder);
            size_t to_copy = std::min(space, raw.size() - offset);
            memcpy(an_decoder_pointer(&decoder), raw.data() + offset, to_copy);
            an_decoder_increment(&decoder, to_copy);
            offset += to_copy;

            while ((packet = an_packet_decode(&decoder)) != nullptr) {
                if (packet->id == packet_id_system_state) {
                    system_state_packet_t ssp;
                    if (decode_system_state_packet(&ssp, packet) == 0) {
                        // Convert to NED and store as waypoint
                        // (reference will be set on first RTK fix in read())
                        // Store raw geodetic for now, convert later
                        waypoints_.push_back({
                            ssp.latitude,
                            ssp.longitude,
                            ssp.height
                        });
                    }
                }
                an_packet_free(&packet);
            }
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from file", waypoints_.size());

        if (waypoints_.empty()) {
            RCLCPP_ERROR(get_logger(), "No waypoints found in file");
            return CallbackReturn::ERROR;
        }
    }
#if GPS_ACTIVE
    // Live mode — open socket and start thread as before
    openSocket();
    if (socket_fd_ < 0) {
        RCLCPP_ERROR(get_logger(), "Cannot activate: socket not connected");
        return CallbackReturn::ERROR;
    }
    running_ = true;
    reader_thread_ = std::thread(&GpsHardware::ethernetLoop, this);
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
    
    if (log_file_ != nullptr)    
    {
        fflush(log_file_);
        fclose(log_file_);
    }

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
        state_interfaces.emplace_back(n, "next_point_north", &joint.state.next_point_north );
        state_interfaces.emplace_back(n, "next_point_east", &joint.state.next_point_east );
        state_interfaces.emplace_back(n, "next_point_down", &joint.state.next_point_down );
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

    if (read_)
    {
        if (!isRefInit) 
        {
            geodetic_converter_.initialiseReference(copy.latitude, copy.longitude, copy.height);
            geodetic_converter_.geodetic2Ned(copy.latitude, copy.longitude, copy.height,
                &north_, &east_, &down_);
            
            if (getNextReferencePoint({north_, east_, down_}, next_waypoint_))
            {
                RCLCPP_INFO(get_logger(), "Initial reference set to: north=%.2f east=%.2f down=%.2f", north_, east_, down_);
                isRefInit = true;
            }
            else
            {
                RCLCPP_WARN(get_logger(), "No valid initial waypoint found. Waiting for a valid reference point...");
            }

        }
        else 
        {
            geodetic_converter_.geodetic2Ned(copy.latitude, copy.longitude, copy.height, &north_, &east_, &down_);
            if (getNextReferencePoint({north_, east_, down_}, next_waypoint_))
            {
                //RCLCPP_INFO(get_logger(), "Next waypoint: north=%.2f east=%.2f down=%.2f", next_waypoint_[0], next_waypoint_[1], next_waypoint_[2]);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "No more waypoints found. Holding position at: north=%.2f east=%.2f down=%.2f", north_, east_, down_);
            }
        }
    }
#else

    if (read_)
    {
        if (!isRefInit) 
        {
            geodetic_converter_.initialiseReference(waypoints_[0][0], waypoints_[0][1], waypoints_[0][2]);
            geodetic_converter_.geodetic2Ned(waypoints_[0][0], waypoints_[0][1], waypoints_[0][2],
                &north_, &east_, &down_);
            
            if (getNextReferencePoint({north_, east_, down_}, next_waypoint_))
            {
                RCLCPP_INFO(get_logger(), "Initial reference set to: north=%.2f east=%.2f down=%.2f", north_, east_, down_);
                curr_north_ = north_;
                curr_east_ = east_;
                curr_down_ = down_;
                isRefInit = true;
            }
            else
            {
                RCLCPP_WARN(get_logger(), "No valid initial waypoint found. Waiting for a valid reference point...");
            }

        }
        else 
        {
            if (getNextReferencePoint({north_, east_, down_}, next_waypoint_))
            {
                if (id_for_read_no_gps_ % 20 == 0)
                {
                    RCLCPP_INFO(get_logger(), "Next waypoint: north=%.2f east=%.2f down=%.2f", next_waypoint_[0], next_waypoint_[1], next_waypoint_[2]);
                    north_ = next_waypoint_[0];
                    east_ = next_waypoint_[1];
                    down_ = next_waypoint_[2];
                    id_for_read_no_gps_ = 0;
                }
                id_for_read_no_gps_++;

            }
            else
            {
                RCLCPP_INFO(get_logger(), "No more waypoints found. Holding position at: north=%.2f east=%.2f down=%.2f", north_, east_, down_);
            }
        }
    }

#endif

    for (auto &joint : joints_) 
    {
        joint.state = copy;
        joint.state.next_point_north = next_waypoint_[0];
        joint.state.next_point_east = next_waypoint_[1];
        joint.state.next_point_down = next_waypoint_[2];
        if (i % 100 == 0)
        {
//            RCLCPP_INFO(get_logger(), "Writing command for %s: gnss_fix=%.2f", joint.joint_name.c_str(), joint.state.gnss_fix);
//            RCLCPP_INFO(get_logger(), "Writing command for %s: acc=%.2f", joint.joint_name.c_str(), joint.state.accelerometer_y);
//            RCLCPP_INFO(get_logger(), "Writing command for %s: rollx=%.2f", joint.joint_name.c_str(), joint.state.roll);
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

bool GpsHardware::getNextReferencePoint(std::array<double, 3> current_point, std::array<double, 3> &next_point)
{
    bool waypoint_found = false;

    if(!isRefInit)
    {
        for (size_t m = 0; m < waypoints_.size(); m++)
        {
            auto waypoint = waypoints_[m];
            double n, e, d;
            geodetic_converter_.geodetic2Ned(waypoint[0], waypoint[1], waypoint[2], &n, &e, &d);
            waypoints_[m] = {n, e, d};
        }
    }
    
    while (waypoint_id_ <= waypoints_.size())
    {
        double distance = pow(waypoints_[waypoint_id_][0] - current_point[0], 2) + pow(waypoints_[waypoint_id_][1] - current_point[1], 2) + pow(waypoints_[waypoint_id_][2] - current_point[2], 2);
        distance = sqrt(distance) - WAYPOINT_MIN_DIST_SQ;
        if (distance > 0)
        {
            next_point = waypoints_[waypoint_id_];
            waypoint_found = true;
            break;
        }
        waypoint_id_++;
    }
    return waypoint_found;
}
} // namespace example

PLUGINLIB_EXPORT_CLASS(example::GpsHardware, hardware_interface::SensorInterface)
