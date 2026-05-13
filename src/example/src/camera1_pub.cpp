#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <time.h>       // clock_gettime, CLOCK_MONOTONIC

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

// ── must match the subscriber setting ────────────────────────────────────────
// When USE_CLOCK_MONOTONIC = 1 in the subscriber, set it to 1 here too.
// Both sides must use the same clock or diffs will be meaningless.
#define USE_CLOCK_MONOTONIC   1
#define REALSENSE 0

// ── globals for signal handling ──────────────────────────────────────────────
std::atomic<bool> stop{false};

void signal_handler(int) { stop = true; }

// ── helper: read CLOCK_MONOTONIC as nanoseconds ───────────────────────────────
inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

// ── constants — must match camera and fixed_size_msgs layout ─────────────────
constexpr int      CAM_INDEX   = 2;
constexpr int      IMG_WIDTH   = 1280;
constexpr int      IMG_HEIGHT  = 720;
#if REALSENSE
constexpr int      IMG_TYPE_DEPTH    = CV_16UC1;      
constexpr int      IMG_TYPE_COLOR    = CV_8UC3;         // bgr8, 3 bytes per pixel
constexpr size_t   PIXEL_BYTES_DEPTH = 2;
constexpr size_t   PIXEL_BYTES_COLOR = 3;
#else
constexpr int      IMG_TYPE    = CV_8UC3;         // bgr8, 3 bytes per pixel
constexpr size_t   PIXEL_BYTES = 3;
#endif
constexpr uint32_t CAM_FREQ_HZ = 30;              // set to known camera frequency

int main(int argc, char ** argv)
{
    // ── signal setup ─────────────────────────────────────────────────────────
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── ROS2 init ────────────────────────────────────────────────────────────
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_publisher_node");

    // ── QoS: must be best_effort + volatile for iceoryx zero-copy ────────────
    auto qos = rclcpp::QoS(1)
        .best_effort()
        .durability_volatile();

#if REALSENSE
    auto pub_color = node->create_publisher<Image8Mb>("camera", qos);
    auto pub_depth = node->create_publisher<Image8Mb>("camera_2", qos);


    rs2::pipeline p;
    rs2::config cfg;
    //The depth is meters is equal to depth scale * pixel value.
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    p.start(cfg);
    
    // get depth scale for metadata
    rs2::depth_sensor depth_sensor = p.get_active_profile()
        .get_device()
        .first<rs2::depth_sensor>();
    float depth_scale = depth_sensor.get_depth_scale();
    RCLCPP_INFO(node->get_logger(), "Depth scale: %.6f", depth_scale);

    const size_t step_depth = IMG_WIDTH * PIXEL_BYTES_DEPTH;
    const size_t step_color = IMG_WIDTH * PIXEL_BYTES_COLOR;

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

    const size_t step       = IMG_WIDTH * PIXEL_BYTES;
    const size_t frame_size = step * IMG_HEIGHT;

    {
        // Borrow once just to check pool chunk is large enough, then discard.
        // data is a fixed C array uint8[12582912] — use sizeof, not .size()
        auto probe = pub->borrow_loaned_message();
        constexpr size_t buf_size = sizeof(probe.get().data);
        if (frame_size > buf_size) {
            RCLCPP_ERROR(
                node->get_logger(),
                "Frame size %zu exceeds message buffer %zu. "
                "Check fixed_size_msgs definition.",
                frame_size, buf_size);
            return 1;
        }
    }  // probe is returned to pool here

#endif

    // ── pre-compute sizes and validate against fixed message buffer ───────────

    RCLCPP_INFO(node->get_logger(), "Camera publisher started — zero-copy path");

    // ── main capture + publish loop ───────────────────────────────────────────
    while (!stop && rclcpp::ok())
    {
        
#if REALSENSE
        // ── Step 1: borrow a loaned chunk from iceoryx ────────────────────────
        auto loaned_msg_depth = pub_depth->borrow_loaned_message();
        auto loaned_msg_color = pub_color->borrow_loaned_message();
        auto & msg_depth      = loaned_msg_depth.get();
        auto & msg_color      = loaned_msg_color.get();

        rs2::frameset frames   = p.wait_for_frames();
        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::video_frame color = frames.get_color_frame();

        cv::Mat depth_frame(
            IMG_HEIGHT,
            IMG_WIDTH,
            IMG_TYPE_DEPTH,
            msg_depth.data.data()
        );

        cv::Mat color_frame(
            IMG_HEIGHT,
            IMG_WIDTH,
            IMG_TYPE_COLOR,
            msg_color.data.data()
        );

        
        const int64_t capture_ns = monotonic_now_ns();
        
        RCLCPP_INFO(node->get_logger(), "data size : %lu", color.get_data_size());

        memcpy(depth_frame.data, depth.get_data(), depth.get_height() * depth.get_stride_in_bytes());
        memcpy(color_frame.data, color.get_data(), color.get_height() * color.get_stride_in_bytes());

        msg_depth.timestamp    = capture_ns;
        msg_depth.width        = IMG_WIDTH;
        msg_depth.height       = IMG_HEIGHT;
        msg_depth.step         = static_cast<uint32_t>(step_depth);
        msg_depth.is_bigendian = false;
        msg_depth.frequency    = CAM_FREQ_HZ;

        msg_color.timestamp    = capture_ns;
        msg_color.width        = IMG_WIDTH;
        msg_color.height       = IMG_HEIGHT;
        msg_color.step         = static_cast<uint32_t>(step_color);
        msg_color.is_bigendian = false;
        msg_color.frequency    = CAM_FREQ_HZ;

        const int64_t time_ns = monotonic_now_ns();   

        msg_depth.publish_timestamp = time_ns;
        msg_color.publish_timestamp = time_ns;

        pub_depth->publish(std::move(loaned_msg_depth));
        pub_color->publish(std::move(loaned_msg_color));

#else
        // ── Step 2: construct a cv::Mat header over iceoryx shared memory ─────
        //    No allocation — OpenCV will write directly into the loaned buffer.
        //    Width, height and type must be known before read() is called.
        auto loaned_msg = pub->borrow_loaned_message();
        auto & msg      = loaned_msg.get();

        cv::Mat frame(
            IMG_HEIGHT,
            IMG_WIDTH,
            IMG_TYPE,
            msg.data.data()        // ← raw C array, no .data() needed
        );

        // ── Step 3: capture directly into iceoryx memory (zero-copy) ──────────
        if (!cap.read(frame)) {
            RCLCPP_WARN(node->get_logger(), "Blank frame — skipping");
            // loaned_msg destructor returns chunk to pool automatically
            continue;
        }

        // ── Step 4: fill metadata ─────────────────────────────────────────────
        // Timestamp sampled immediately after read() — closest to capture time.
        // Must use the same clock as the subscriber — both USE_CLOCK_MONOTONIC
        // must be set to the same value or latency diffs will be wrong.
#if USE_CLOCK_MONOTONIC
        msg.timestamp    = monotonic_now_ns();
#else
        msg.timestamp    = node->now().nanoseconds();
#endif
        msg.width        = IMG_WIDTH;
        msg.height       = IMG_HEIGHT;
        msg.step         = static_cast<uint32_t>(step);
        msg.is_bigendian = false;
        msg.frequency    = CAM_FREQ_HZ;

        // ── Step 5: publish — iceoryx transfers ownership, no copy ────────────
        // publish_timestamp sampled as late as possible — right before handoff
        // to iceoryx. The interval (publish_timestamp → subscriber receive_ns)
        // isolates pure transport + wakeup jitter from camera capture time.
#if USE_CLOCK_MONOTONIC
        msg.publish_timestamp = monotonic_now_ns();
#else
        msg.publish_timestamp = node->now().nanoseconds();
#endif
        pub->publish(std::move(loaned_msg));

#endif
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down camera publisher");
#if REALSENSE
#else
    cap.release();
#endif
    rclcpp::shutdown();
    return 0;
}