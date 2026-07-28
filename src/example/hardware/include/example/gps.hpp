#pragma once

// SDK headers incluídos apenas com os tipos necessários
// O read_gnss_compass.h NÃO deve ser incluído aqui (tem sockets, statics globais, etc.)
// Inclui-o apenas no gps.cpp
#include "gnss_compass_utils/an_packet_protocol.h"
#include "gnss_compass_utils/ins_packets.h"
#include "example/geodetic_conv.hpp"

#include <realtime_tools/realtime_buffer.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <gpiod.h>

namespace example
{
    // ------------------------------------------------------------
    // Dados lidos do sensor num dado instante
    // ------------------------------------------------------------
    struct GPSData
    {
        double gnss_fix{-1.0};
        double counter{0.0};

        double latitude{0.0};
        double longitude{0.0};
        double height{0.0};

        double velocity_north{0.0};
        double velocity_east{0.0};
        double velocity_down{0.0};

        double body_acceleration_x{0.0};
        double body_acceleration_y{0.0};
        double body_acceleration_z{0.0};

        double g_force{0.0};

        double roll{0.0};
        double pitch{0.0};
        double heading{0.0};

        double angular_velocity_x{0.0};
        double angular_velocity_y{0.0};
        double angular_velocity_z{0.0};

        double standard_deviation_latitude{0.0};
        double standard_deviation_longitude{0.0};
        double standard_deviation_height{0.0};

        double accelerometer_x{0.0};
        double accelerometer_y{0.0};
        double accelerometer_z{0.0};

        double next_point_north{2.0};
        double next_point_east{3.0};
        double next_point_down{4.0};
    };

    // ------------------------------------------------------------
    // Sensor "joint" — mapeia um sensor para os seus state interfaces
    // ------------------------------------------------------------
    struct GpsJoint
    {
        explicit GpsJoint(const std::string &name) : joint_name(name) {}
        GpsJoint() = default;

        std::string joint_name;
        GPSData     state;
    };

    // ------------------------------------------------------------
    // Hardware plugin
    // ------------------------------------------------------------
    class GpsHardware : public hardware_interface::SensorInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(GpsHardware)

        // ── Lifecycle ──────────────────────────────────────────
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override
        {
            RCLCPP_INFO(get_logger(), "Cleaning up GpsHardware");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override
        {
            RCLCPP_INFO(get_logger(), "Shutting down GpsHardware");
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        // ── ros2_control ───────────────────────────────────────
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // ── Helpers ────────────────────────────────────────────
        rclcpp::Clock  get_clock();
        rclcpp::Logger get_logger();
        bool getNextReferencePoint(std::array<double, 3> current_point, std::array<double, 3> &next_point);


    private:
        // ── Rede ───────────────────────────────────────────────
        void openSocket();
        void ethernetLoop();

        int         socket_fd_{-1};
        std::string ip_{"192.168.1.150"};
        int         port_{16718};

        // ── SDK ────────────────────────────────────────────────
        an_decoder_t an_decoder_{};          // inicializado no on_configure
        an_packet_t *an_packet_{nullptr};
        system_state_packet_t system_state_packet_{};
        raw_sensors_packet_t raw_sensors_packet_{};
        int gnss_fix_type_prev_{-1}; // para detetar mudança de tipo de fix
        int bytes_received{0};

        int i{0};

        // ── Thread / dados ─────────────────────────────────────
        std::atomic<bool> running_{false};
        std::thread reader_thread_;
        realtime_tools::RealtimeBuffer<GPSData> latest_data_;

        // ── Joints (sensor interfaces) ──────────────────────────
        std::vector<GpsJoint> joints_;

        rclcpp::Clock clock_;

        std::vector<std::array<double, 3>> waypoints_{};
        geodetic_converter::GeodeticConverter geodetic_converter_;
        bool isRefInit{false};
        double north_{0.0}, east_{0.0}, down_{0.0};
        double curr_north_{0.0}, curr_east_{0.0}, curr_down_{0.0};
        int waypoint_id_{0};
        int id_for_read_no_gps_{0};
        std::array<double, 3> next_waypoint_{};

        FILE *log_file_{nullptr};
        int flush_counter_{0};
        std::string file_name_to_reproduce_{""};
        int read_{0};

        GPSData copy{};
        //gpiod_chip *chip;
        //gpiod_line *line;
    };

} // namespace example
