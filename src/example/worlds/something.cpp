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

    open_socket();
    reader_thread_ = std::thread(&GpsHardware::ethernet_loop, this);

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GpsHardware::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    running_ = false;

    if (reader_thread_.joinable())
        reader_thread_.join();

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

        void ethernet_loop();

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
        std::string interface_type_ =  "ethernet";

        // ethernet
        int socket_fd_ = -1;
        std::string ip_ = "192.168.1.10";
        int port_ = 50010;
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


// file created by cyrusbehr at 04/2024
// adapted by G-S-Rodrigues 06/2024

#include "yolov8.h"
#include "header_accessor.h"
#include "utils.h"
#include <opencv2/cudaimgproc.hpp>

YoloV8::YoloV8(const std::string &onnxModelPath, const YoloV8Config &config)
    : PROBABILITY_THRESHOLD(config.probabilityThreshold), NMS_THRESHOLD(config.nmsThreshold), TOP_K(config.topK),
      SEG_CHANNELS(config.segChannels), SEG_H(config.segH), SEG_W(config.segW), SEGMENTATION_THRESHOLD(config.segmentationThreshold),
      CLASS_NAMES(config.classNames), NUM_KPS(config.numKPS), KPS_THRESHOLD(config.kpsThreshold)
{
    // Specify options for GPU inference
    Options options;
    options.optBatchSize = 1;
    options.maxBatchSize = 1;

    options.precision = config.precision;
    options.calibrationDataDirectoryPath = config.calibrationDataDirectory;

    if (options.precision == Precision::INT8)
    {
        if (options.calibrationDataDirectoryPath.empty())
        {
            throw std::runtime_error("Error: Must supply calibration data path for INT8 calibration");
        }
    }

    // Create our TensorRT inference engine
    m_trtEngine = std::make_unique<Engine<float>>(options);

    // Build the onnx model into a TensorRT engine file, cache the file to disk, and then load the TensorRT engine file into memory.
    // If the engine file already exists on disk, this function will not rebuild but only load into memory.
    // The engine file is rebuilt any time the above Options are changed.
    auto succ = m_trtEngine->buildLoadNetwork(onnxModelPath, SUB_VALS, DIV_VALS, NORMALIZE);
    if (!succ)
    {
        const std::string errMsg = "Error: Unable to build or load the TensorRT engine. "
                                   "Try increasing TensorRT log severity to kVERBOSE (in /libs/tensorrt-cpp-api/engine.cpp).";
        throw std::runtime_error(errMsg);
    }
}

std::vector<std::vector<cv::cuda::GpuMat>> YoloV8::preprocess(const cv::cuda::GpuMat &gpuImg)
{
    // Populate the input vectors
    const auto &inputDims = m_trtEngine->getInputDims();

    // Convert the image from BGR to RGB
    cv::cuda::GpuMat rgbMat;
    cv::cuda::cvtColor(gpuImg, rgbMat, cv::COLOR_BGR2RGB);

    auto resized = rgbMat;

    // Resize to the model expected input size while maintaining the aspect ratio with the use of padding
    if (resized.rows != inputDims[0].d[1] || resized.cols != inputDims[0].d[2])
    {
        // Only resize if not already the right size to avoid unecessary copy
        resized = Engine<float>::resizeKeepAspectRatioPadRightBottom(rgbMat, inputDims[0].d[1], inputDims[0].d[2]);
    }

    // Convert to format expected by our inference engine
    // The reason for the strange format is because it supports models with multiple inputs as well as batching
    // In our case though, the model only has a single input and we are using a batch size of 1.
    std::vector<cv::cuda::GpuMat> input{std::move(resized)};
    std::vector<std::vector<cv::cuda::GpuMat>> inputs{std::move(input)};

    // These params will be used in the post-processing stage
    m_imgHeight = rgbMat.rows;
    m_imgWidth = rgbMat.cols;
    m_ratio = 1.f / std::min(inputDims[0].d[2] / static_cast<float>(rgbMat.cols), inputDims[0].d[1] / static_cast<float>(rgbMat.rows));

    return inputs;
}

std::vector<Object> YoloV8::detectObjects(const cv::cuda::GpuMat &inputImageBGR)
{
    // Preprocess the input image
#ifdef ENABLE_BENCHMARKS
    static int numIts = 1;
    preciseStopwatch s1;
#endif
    const auto input = preprocess(inputImageBGR);
#ifdef ENABLE_BENCHMARKS
    static long long t1 = 0;
    t1 += s1.elapsedTime<long long, std::chrono::microseconds>();
    std::cout << "Avg Preprocess time: " << (t1 / numIts) / 1000.f << " ms" << std::endl;
#endif
    // Run inference using the TensorRT engine
#ifdef ENABLE_BENCHMARKS
    preciseStopwatch s2;
#endif
    std::vector<std::vector<std::vector<float>>> featureVectors;
    auto succ = m_trtEngine->runInference(input, featureVectors);
    if (!succ)
    {
        throw std::runtime_error("Error: Unable to run inference.");
    }
#ifdef ENABLE_BENCHMARKS
    static long long t2 = 0;
    t2 += s2.elapsedTime<long long, std::chrono::microseconds>();
    std::cout << "Avg Inference time: " << (t2 / numIts) / 1000.f << " ms" << std::endl;
    preciseStopwatch s3;
#endif
    // Check if our model does only object detection or also supports segmentation
    std::vector<Object> ret;
    const auto &numOutputs = m_trtEngine->getOutputDims().size();
    if (numOutputs == 1)
    {
        // Object detection or pose estimation
        // Since we have a batch size of 1 and only 1 output, we must convert the output from a 3D array to a 1D array.
        std::vector<float> featureVector;
        Engine<float>::transformOutput(featureVectors, featureVector);

        const auto &outputDims = m_trtEngine->getOutputDims();
        int numChannels = outputDims[outputDims.size() - 1].d[1];
        // TODO: Need to improve this to make it more generic (don't use magic number).
        // For now it works with Ultralytics pretrained models.
        if (numChannels == 56)
        {
            // Pose estimation
            ret = postprocessPose(featureVector);
        }
        else
        {
            // Object detection
            ret = postprocessDetect(featureVector);
        }
    }
    else
    {
        // Segmentation
        // Since we have a batch size of 1 and 2 outputs, we must convert the output from a 3D array to a 2D array.
        std::vector<std::vector<float>> featureVector;
        Engine<float>::transformOutput(featureVectors, featureVector);
        ret = postProcessSegmentation(featureVector);
    }
#ifdef ENABLE_BENCHMARKS
    static long long t3 = 0;
    t3 += s3.elapsedTime<long long, std::chrono::microseconds>();
    std::cout << "Avg Postprocess time: " << (t3 / numIts++) / 1000.f << " ms\n"
              << std::endl;
#endif
    return ret;
}

std::vector<Object> YoloV8::detectObjects(const cv::Mat &inputImageBGR)
{
    // Upload the image to GPU memory
    cv::cuda::GpuMat gpuImg;
    gpuImg.upload(inputImageBGR);

    // Call detectObjects with the GPU image
    return detectObjects(gpuImg);
}

std::vector<Object> YoloV8::extractObjects(const cv::Mat &depth_img, std::vector<Object> objects, image_intrinsics image_intrinsics, float maximum_detection_threshold)
{
    // Extract depth data for each object and erase the ones without depth information
    for (auto it = objects.begin(); it != objects.end();)
    {
        auto &obj = *it; // Reference to the current object

        int mask_height_selected;
        cv::Point offset;
        if (obj.label == 0)
        {
            // std::cout << "Object is a person!" << std::endl;
            mask_height_selected = obj.rect.height;
            offset = cv::Point(obj.rect.x, obj.rect.y);
        }
        else
        {
            mask_height_selected = obj.rect.height / 3;
            offset = cv::Point(obj.rect.x, (obj.rect.y + obj.rect.height * 2 / 3));
        }

        cv::Mat mask_selected = obj.boxMask(cv::Rect(0, obj.rect.height - mask_height_selected, obj.boxMask.cols, mask_height_selected));

        std::vector<cv::Point> mask_indices;
        cv::findNonZero(mask_selected, mask_indices);

        for (cv::Point &point : mask_indices)
        {
            point.x += offset.x;
            point.y += offset.y;
        }

        // calculate the x and y position in 2D for image representation
        cv::Point xy_position;
        for (const cv::Point &point : mask_indices)
        {
            xy_position.x += point.x;
            xy_position.y += point.y;
        }
        xy_position.x /= mask_indices.size();
        xy_position.y /= mask_indices.size();
        obj.Pose2D = xy_position;

        // extract depth from the selected area
        std::vector<uint16_t> depth_values;
        for (const cv::Point &point : mask_indices)
        {
            depth_values.push_back(depth_img.at<uint16_t>(point));
        }

        // check if depth values are different than 0
        bool has_non_zero = std::any_of(depth_values.begin(), depth_values.end(), [](uint16_t value)
                                        { return value != 0; });

        if (!has_non_zero)
        {
            it = objects.erase(it);
            continue;
        }
        // filter depth
        size_t zero_count = std::count(depth_values.begin(), depth_values.end(), 0);
        depth_values.erase(std::remove(depth_values.begin(), depth_values.end(), 0), depth_values.end());
        uint16_t depth_median = computeMedian(depth_values);
        float threshold = maximum_detection_threshold / image_intrinsics.depth_units;
        filter_depth_threshold(depth_values, depth_median, threshold);
        if (depth_values.empty())
        {
            it = objects.erase(it);
            continue;
        }
        float depth_mean = computeMean(depth_values);

        float z_world = depth_mean * image_intrinsics.depth_units;
        float x_world = z_world * (xy_position.x - image_intrinsics.ppx) / image_intrinsics.fx;
        float y_world = z_world * (xy_position.y - image_intrinsics.ppy) / image_intrinsics.fy;

        std::vector<float> pose3D_values = {x_world, y_world, z_world};
        obj.Pose3D = pose3D_values;

        ++it;
    }
    return objects;
}

std::vector<Object> YoloV8::postProcessSegmentation(std::vector<std::vector<float>> &featureVectors)
{
    const auto &outputDims = m_trtEngine->getOutputDims();

    int numChannels = outputDims[0].d[1];
    int numAnchors = outputDims[0].d[2];

    const auto numClasses = numChannels - SEG_CHANNELS - 4;

    // Ensure the output lengths are correct
    if (featureVectors[0].size() != static_cast<size_t>(numChannels) * numAnchors)
    {
        throw std::logic_error("Output at index 0 has incorrect length");
    }

    if (featureVectors[1].size() != static_cast<size_t>(SEG_CHANNELS) * SEG_H * SEG_W)
    {
        throw std::logic_error("Output at index 1 has incorrect length");
    }

    cv::Mat output = cv::Mat(numChannels, numAnchors, CV_32F, featureVectors[0].data());
    output = output.t();

    cv::Mat protos = cv::Mat(SEG_CHANNELS, SEG_H * SEG_W, CV_32F, featureVectors[1].data());

    std::vector<int> labels;
    std::vector<float> scores;
    std::vector<cv::Rect> bboxes;
    std::vector<cv::Mat> maskConfs;
    std::vector<int> indices;

    // Object the bounding boxes and class labels
    for (int i = 0; i < numAnchors; i++)
    {
        auto rowPtr = output.row(i).ptr<float>();
        auto bboxesPtr = rowPtr;
        auto scoresPtr = rowPtr + 4;
        auto maskConfsPtr = rowPtr + 4 + numClasses;
        auto maxSPtr = std::max_element(scoresPtr, scoresPtr + numClasses);
        float score = *maxSPtr;
        if (score > PROBABILITY_THRESHOLD)
        {
            float x = *bboxesPtr++;
            float y = *bboxesPtr++;
            float w = *bboxesPtr++;
            float h = *bboxesPtr;

            float x0 = std::clamp((x - 0.5f * w) * m_ratio, 0.f, m_imgWidth);
            float y0 = std::clamp((y - 0.5f * h) * m_ratio, 0.f, m_imgHeight);
            float x1 = std::clamp((x + 0.5f * w) * m_ratio, 0.f, m_imgWidth);
            float y1 = std::clamp((y + 0.5f * h) * m_ratio, 0.f, m_imgHeight);

            int label = maxSPtr - scoresPtr;
            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;

            cv::Mat maskConf = cv::Mat(1, SEG_CHANNELS, CV_32F, maskConfsPtr);

            bboxes.push_back(bbox);
            labels.push_back(label);
            scores.push_back(score);
            maskConfs.push_back(maskConf);
        }
    }

    // Require OpenCV 4.7 for this function
    cv::dnn::NMSBoxesBatched(bboxes, scores, labels, PROBABILITY_THRESHOLD, NMS_THRESHOLD, indices);

    // Obtain the segmentation masks
    cv::Mat masks;
    std::vector<Object> objs;
    int cnt = 0;
    for (auto &i : indices)
    {
        if (cnt >= TOP_K)
        {
            break;
        }
        cv::Rect tmp = bboxes[i];
        Object obj;
        obj.label = labels[i];
        obj.rect = tmp;
        obj.probability = scores[i];
        masks.push_back(maskConfs[i]);
        objs.push_back(obj);
        cnt += 1;
    }

    // Convert segmentation mask to original frame
    if (!masks.empty())
    {
        cv::Mat matmulRes = (masks * protos).t();
        cv::Mat maskMat = matmulRes.reshape(indices.size(), {SEG_W, SEG_H});

        std::vector<cv::Mat> maskChannels;
        cv::split(maskMat, maskChannels);
        const auto inputDims = m_trtEngine->getInputDims();

        cv::Rect roi;
        if (m_imgHeight > m_imgWidth)
        {
            roi = cv::Rect(0, 0, SEG_W * m_imgWidth / m_imgHeight, SEG_H);
        }
        else
        {
            roi = cv::Rect(0, 0, SEG_W, SEG_H * m_imgHeight / m_imgWidth);
        }

        for (size_t i = 0; i < indices.size(); i++)
        {
            cv::Mat dest, mask;
            cv::exp(-maskChannels[i], dest);
            dest = 1.0 / (1.0 + dest);
            dest = dest(roi);
            cv::resize(dest, mask, cv::Size(static_cast<int>(m_imgWidth), static_cast<int>(m_imgHeight)), cv::INTER_LINEAR);
            objs[i].boxMask = mask(objs[i].rect) > SEGMENTATION_THRESHOLD;
        }
    }

    return objs;
}

std::vector<Object> YoloV8::postprocessPose(std::vector<float> &featureVector)
{
    const auto &outputDims = m_trtEngine->getOutputDims();
    auto numChannels = outputDims[0].d[1];
    auto numAnchors = outputDims[0].d[2];

    std::vector<cv::Rect> bboxes;
    std::vector<float> scores;
    std::vector<int> labels;
    std::vector<int> indices;
    std::vector<std::vector<float>> kpss;

    cv::Mat output = cv::Mat(numChannels, numAnchors, CV_32F, featureVector.data());
    output = output.t();

    // Get all the YOLO proposals
    for (int i = 0; i < numAnchors; i++)
    {
        auto rowPtr = output.row(i).ptr<float>();
        auto bboxesPtr = rowPtr;
        auto scoresPtr = rowPtr + 4;
        auto kps_ptr = rowPtr + 5;
        float score = *scoresPtr;
        if (score > PROBABILITY_THRESHOLD)
        {
            float x = *bboxesPtr++;
            float y = *bboxesPtr++;
            float w = *bboxesPtr++;
            float h = *bboxesPtr;

            float x0 = std::clamp((x - 0.5f * w) * m_ratio, 0.f, m_imgWidth);
            float y0 = std::clamp((y - 0.5f * h) * m_ratio, 0.f, m_imgHeight);
            float x1 = std::clamp((x + 0.5f * w) * m_ratio, 0.f, m_imgWidth);
            float y1 = std::clamp((y + 0.5f * h) * m_ratio, 0.f, m_imgHeight);

            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;

            std::vector<float> kps;
            for (int k = 0; k < NUM_KPS; k++)
            {
                float kpsX = *(kps_ptr + 3 * k) * m_ratio;
                float kpsY = *(kps_ptr + 3 * k + 1) * m_ratio;
                float kpsS = *(kps_ptr + 3 * k + 2);
                kpsX = std::clamp(kpsX, 0.f, m_imgWidth);
                kpsY = std::clamp(kpsY, 0.f, m_imgHeight);
                kps.push_back(kpsX);
                kps.push_back(kpsY);
                kps.push_back(kpsS);
            }

            bboxes.push_back(bbox);
            labels.push_back(0); // All detected objects are people
            scores.push_back(score);
            kpss.push_back(kps);
        }
    }

    // Run NMS
    cv::dnn::NMSBoxesBatched(bboxes, scores, labels, PROBABILITY_THRESHOLD, NMS_THRESHOLD, indices);

    std::vector<Object> objects;

    // Choose the top k detections
    int cnt = 0;
    for (auto &chosenIdx : indices)
    {
        if (cnt >= TOP_K)
        {
            break;
        }

        Object obj{};
        obj.probability = scores[chosenIdx];
        obj.label = labels[chosenIdx];
        obj.rect = bboxes[chosenIdx];
        obj.kps = kpss[chosenIdx];
        objects.push_back(obj);

        cnt += 1;
    }

    return objects;
}

std::vector<Object> YoloV8::postprocessDetect(std::vector<float> &featureVector)
{
    const auto &outputDims = m_trtEngine->getOutputDims();
    auto numChannels = outputDims[0].d[1];
    auto numAnchors = outputDims[0].d[2];

    auto numClasses = CLASS_NAMES.size();

    std::vector<cv::Rect> bboxes;
    std::vector<float> scores;
    std::vector<int> labels;
    std::vector<int> indices;

    cv::Mat output = cv::Mat(numChannels, numAnchors, CV_32F, featureVector.data());
    output = output.t();

    // Get all the YOLO proposals
    for (int i = 0; i < numAnchors; i++)
    {
        auto rowPtr = output.row(i).ptr<float>();
        auto bboxesPtr = rowPtr;
        auto scoresPtr = rowPtr + 4;
        auto maxSPtr = std::max_element(scoresPtr, scoresPtr + numClasses);
        float score = *maxSPtr;
        if (score > PROBABILITY_THRESHOLD)
        {
            float x = *bboxesPtr++;
            float y = *bboxesPtr++;
            float w = *bboxesPtr++;
            float h = *bboxesPtr;

            float x0 = std::clamp((x - 0.5f * w) * m_ratio, 0.f, m_imgWidth);
            float y0 = std::clamp((y - 0.5f * h) * m_ratio, 0.f, m_imgHeight);
            float x1 = std::clamp((x + 0.5f * w) * m_ratio, 0.f, m_imgWidth);
            float y1 = std::clamp((y + 0.5f * h) * m_ratio, 0.f, m_imgHeight);

            int label = maxSPtr - scoresPtr;
            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;

            bboxes.push_back(bbox);
            labels.push_back(label);
            scores.push_back(score);
        }
    }

    // Run NMS
    cv::dnn::NMSBoxesBatched(bboxes, scores, labels, PROBABILITY_THRESHOLD, NMS_THRESHOLD, indices);

    std::vector<Object> objects;

    // Choose the top k detections
    int cnt = 0;
    for (auto &chosenIdx : indices)
    {
        if (cnt >= TOP_K)
        {
            break;
        }

        Object obj{};
        obj.probability = scores[chosenIdx];
        obj.label = labels[chosenIdx];
        obj.rect = bboxes[chosenIdx];
        objects.push_back(obj);

        cnt += 1;
    }

    return objects;
}

void YoloV8::drawObjectLabels(cv::Mat &image, const std::vector<Object> &objects, unsigned int scale)
{
    // If segmentation information is present, start with that
    if (!objects.empty() && !objects[0].boxMask.empty())
    {
        cv::Mat mask = image.clone();
        for (const auto &object : objects)
        {
            // Choose the color
            int colorIndex = object.label % COLOR_LIST.size(); // We have only defined 80 unique colors
            cv::Scalar color = cv::Scalar(COLOR_LIST[colorIndex][0], COLOR_LIST[colorIndex][1], COLOR_LIST[colorIndex][2]);

            // Add the mask for said object
            mask(object.rect).setTo(color * 255, object.boxMask);
        }
        // Add all the masks to our image
        cv::addWeighted(image, 0.5, mask, 0.5, 1, image);
    }

    // Bounding boxes and annotations
    for (auto &object : objects)
    {
        // Choose the color
        int colorIndex = object.label % COLOR_LIST.size(); // We have only defined 80 unique colors
        cv::Scalar color = cv::Scalar(COLOR_LIST[colorIndex][0], COLOR_LIST[colorIndex][1], COLOR_LIST[colorIndex][2]);
        float meanColor = cv::mean(color)[0];
        cv::Scalar txtColor;
        if (meanColor > 0.5)
        {
            txtColor = cv::Scalar(0, 0, 0);
        }
        else
        {
            txtColor = cv::Scalar(255, 255, 255);
        }

        const auto &rect = object.rect;

        // Draw rectangles and text
        char text[256];
        sprintf(text, "%s %.1f%%", CLASS_NAMES[object.label].c_str(), object.probability * 100);

        int baseLine = 0;
        cv::Size labelSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.3 * scale, scale, &baseLine);

        cv::Scalar txt_bk_color = color * 0.7 * 255;

        int x = object.rect.x;
        int y = object.rect.y + 1;

        cv::rectangle(image, rect, color * 255, scale + 1);

        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(labelSize.width, labelSize.height + baseLine)), txt_bk_color, -1);

        cv::putText(image, text, cv::Point(x, y + labelSize.height), cv::FONT_HERSHEY_SIMPLEX, 0.3 * scale, txtColor, scale);

        if (object.Pose2D != cv::Point(0, 0))
        {
            cv::circle(image, object.Pose2D, 5, txtColor, cv::FILLED);
        }

        // Display x y z world coordinates
        if (!object.Pose3D.empty())
        {
            sprintf(text, "x= %.2f", object.Pose3D[0]);
            cv::Size labelSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.3 * scale, scale, &baseLine);
            cv::rectangle(image, cv::Rect(cv::Point(x + object.rect.width, y + object.rect.height - 3 * labelSize.height), cv::Size(labelSize.width, 3 * labelSize.height + baseLine)), txt_bk_color, -1);
            cv::putText(image, text, cv::Point(x + object.rect.width, y + object.rect.height - 2 * labelSize.height), cv::FONT_HERSHEY_SIMPLEX, 0.25 * scale, txtColor, scale);
            sprintf(text, "y= %.2f", object.Pose3D[1]);
            cv::putText(image, text, cv::Point(x + object.rect.width, y + object.rect.height - labelSize.height), cv::FONT_HERSHEY_SIMPLEX, 0.25 * scale, txtColor, scale);
            sprintf(text, "z= %.2f", object.Pose3D[2]);
            cv::putText(image, text, cv::Point(x + object.rect.width, y + object.rect.height), cv::FONT_HERSHEY_SIMPLEX, 0.25 * scale, txtColor, scale);
        }
        // Pose estimation
        if (!object.kps.empty())
        {
            auto &kps = object.kps;
            for (int k = 0; k < NUM_KPS + 2; k++)
            {
                if (k < NUM_KPS)
                {
                    int kpsX = std::round(kps[k * 3]);
                    int kpsY = std::round(kps[k * 3 + 1]);
                    float kpsS = kps[k * 3 + 2];
                    if (kpsS > KPS_THRESHOLD)
                    {
                        cv::Scalar kpsColor = cv::Scalar(KPS_COLORS[k][0], KPS_COLORS[k][1], KPS_COLORS[k][2]);
                        cv::circle(image, {kpsX, kpsY}, 5, kpsColor, -1);
                    }
                }
                auto &ske = SKELETON[k];
                int pos1X = std::round(kps[(ske[0] - 1) * 3]);
                int pos1Y = std::round(kps[(ske[0] - 1) * 3 + 1]);

                int pos2X = std::round(kps[(ske[1] - 1) * 3]);
                int pos2Y = std::round(kps[(ske[1] - 1) * 3 + 1]);

                float pos1S = kps[(ske[0] - 1) * 3 + 2];
                float pos2S = kps[(ske[1] - 1) * 3 + 2];

                if (pos1S > KPS_THRESHOLD && pos2S > KPS_THRESHOLD)
                {
                    cv::Scalar limbColor = cv::Scalar(LIMB_COLORS[k][0], LIMB_COLORS[k][1], LIMB_COLORS[k][2]);
                    cv::line(image, {pos1X, pos1Y}, {pos2X, pos2Y}, limbColor, 2);
                }
            }
        }
    }
}