#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <librealsense2/rs.hpp>


#include <atomic>
#include <csignal>
#include <cstdint>
#include <time.h>

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

#define USE_CLOCK_MONOTONIC 1
#define REALSENSE           0

std::atomic<bool> stop{false};
void signal_handler(int) { stop = true; }

inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

constexpr int      CAM_INDEX   = 2;
constexpr int      IMG_WIDTH   = 640;
constexpr int      IMG_HEIGHT  = 480;
#if REALSENSE
constexpr int      IMG_TYPE_DEPTH    = CV_16UC1;      
constexpr int      IMG_TYPE_COLOR    = CV_8UC3;         // bgr8, 3 bytes per pixel
constexpr size_t   PIXEL_BYTES_DEPTH = 2;
constexpr size_t   PIXEL_BYTES_COLOR = 3;
#else
constexpr int      IMG_TYPE    = CV_8UC3;         // bgr8, 3 bytes per pixel
constexpr size_t   PIXEL_BYTES = 3;
#endif
constexpr uint32_t CAM_FREQ_HZ = 30;

int main(int argc, char ** argv)
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_publisher_node");

    // ── QoS: reliable works with FastDDS, no special constraints ─────────────
    auto qos = rclcpp::QoS(1)
        .best_effort()
        .durability_volatile();

#if REALSENSE
    auto pub_color = node->create_publisher<Image8Mb>("camera/color/image_raw", qos);
    auto pub_depth = node->create_publisher<Image8Mb>("camera/depth", qos);

    rs2::pipeline p;
    rs2::config cfg;
    //The depth is meters is equal to depth scale * pixel value.
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    p.start(cfg);
    
    // get depth scale for metadata
    rs2::depth_sensor depth_sensor = p.get_active_profile()
        .get_device()
        .first<rs2::depth_sensor>();
    float depth_scale = depth_sensor.get_depth_scale();
    RCLCPP_INFO(node->get_logger(), "Depth scale: %.6f", depth_scale);
#else
    auto pub = node->create_publisher<Image8Mb>("camera", qos);

    // ── camera init ──────────────────────────────────────────────────────────
    cv::VideoCapture cap(CAM_INDEX, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open camera index %d", CAM_INDEX);
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  IMG_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
#endif


#if REALSENSE
    auto msg_color = std::make_unique<Image8Mb>();
    auto msg_depth = std::make_unique<Image8Mb>();

    const size_t step_color = IMG_WIDTH * PIXEL_BYTES_COLOR;
    const size_t step_depth = IMG_WIDTH * PIXEL_BYTES_DEPTH;

    cv::Mat depth_frame(
        IMG_HEIGHT,
        IMG_WIDTH,
        IMG_TYPE_DEPTH,
        msg_depth->data.data()
    );

    cv::Mat color_frame(
        IMG_HEIGHT,
        IMG_WIDTH,
        IMG_TYPE_COLOR,
        msg_color->data.data()
    );

#else
    auto msg = std::make_unique<Image8Mb>();

    const size_t step = IMG_WIDTH * PIXEL_BYTES;

        cv::Mat frame(
            IMG_HEIGHT,
            IMG_WIDTH,
            IMG_TYPE,
            msg->data.data()        // ← raw C array, no .data() needed
        );
#endif

    while (!stop && rclcpp::ok())
    {
        
#if REALSENSE
        rs2::frameset frames = p.wait_for_frames();
        rs2::color_frame color = frames.get_color_frame();
        rs2::depth_frame depth = frames.get_depth_frame();

        memcpy(color_frame.data, color.get_data(), color.get_height() * color.get_stride_in_bytes());
        memcpy(depth_frame.data, depth.get_data(), depth.get_height() * depth.get_stride_in_bytes());

#if USE_CLOCK_MONOTONIC
        const int64_t time_ns = monotonic_now_ns();
#else
        msg->timestamp = node->now().nanoseconds();
#endif

        msg_color->width        = IMG_WIDTH;
        msg_color->height       = IMG_HEIGHT;
        msg_color->step         = static_cast<uint32_t>(step_color);
        msg_color->is_bigendian = false;
        msg_color->frequency    = CAM_FREQ_HZ;
        msg_color->timestamp    = time_ns;

        msg_depth->width        = IMG_WIDTH;
        msg_depth->height       = IMG_HEIGHT;
        msg_depth->step         = static_cast<uint32_t>(step_depth);
        msg_depth->is_bigendian = false;
        msg_depth->frequency    = CAM_FREQ_HZ;
        msg_depth->timestamp    = time_ns;
        
/*
#if USE_CLOCK_MONOTONIC
        msg->publish_timestamp = monotonic_now_ns();
#else
        msg->publish_timestamp = node->now().nanoseconds();
#endif
*/

        pub_color->publish(*msg_color);
        pub_depth->publish(*msg_depth);

#else

        if (!cap.read(frame)) {
            RCLCPP_WARN(node->get_logger(), "Blank frame — skipping");
            // loaned_msg destructor returns chunk to pool automatically
            continue;
        }

        #if USE_CLOCK_MONOTONIC
                const int64_t time_ns = monotonic_now_ns();
        #else
                msg->timestamp = node->now().nanoseconds();
        #endif
        
                msg->width        = IMG_WIDTH;
                msg->height       = IMG_HEIGHT;
                msg->step         = static_cast<uint32_t>(step);
                msg->is_bigendian = false;
                msg->frequency    = CAM_FREQ_HZ;
                msg->timestamp    = time_ns;
                
        #if USE_CLOCK_MONOTONIC
                msg->publish_timestamp = monotonic_now_ns();
        #else
                msg->publish_timestamp = node->now().nanoseconds();
        #endif
        
                pub->publish(*msg);

#endif
        
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down FastDDS publisher");
#if REALSENSE
#else
    cap.release();
#endif
    rclcpp::shutdown();
    return 0;
}
