#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <time.h>

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

#define USE_CLOCK_MONOTONIC 1

std::atomic<bool> stop{false};
void signal_handler(int) { stop = true; }

inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

constexpr int      CAM_INDEX   = 2;
constexpr int      IMG_WIDTH   = 1024;
constexpr int      IMG_HEIGHT  = 1024;
constexpr int      IMG_TYPE    = CV_8UC3;
constexpr size_t   PIXEL_BYTES = 3;
constexpr uint32_t CAM_FREQ_HZ = 30;

int main(int argc, char ** argv)
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_publisher_node");

    auto qos = rclcpp::QoS(1)
        .best_effort()
        .durability_volatile();

    // ── publisher 1 — vision pipeline (iceoryx zero-copy) ────────────────────
    auto pub = node->create_publisher<Image8Mb>("camera", qos);

    // ── camera init ───────────────────────────────────────────────────────────
    cv::VideoCapture cap(CAM_INDEX, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open camera index %d", CAM_INDEX);
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  IMG_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
    cap.set(cv::CAP_PROP_BUFFERSIZE,   1);

    const size_t step = IMG_WIDTH * PIXEL_BYTES;
    const size_t frame_size = step * IMG_HEIGHT;

    // ── validate iceoryx buffer ───────────────────────────────────────────────
    {
        auto probe = pub->borrow_loaned_message();
        constexpr size_t buf_size = sizeof(probe.get().data_color);
        if (frame_size > buf_size) {
            RCLCPP_ERROR(node->get_logger(),
                "Frame size %zu exceeds buffer %zu", frame_size, buf_size);
            return 1;
        }
    }

    // ── pre-allocate nav2 message ─────────────────────────────────────────────
    // allocate once — reuse every frame to avoid heap allocation per frame
    // sensor_msgs/Image data vector is dynamic — only actual image bytes

    RCLCPP_INFO(node->get_logger(),
        "Dual publisher started\n"
        "  /camera             — Image8Mb (iceoryx zero-copy, vision pipeline)\n"
        "  /camera_2 — sensor_msgs/Image (FastDDS, Nav2/RViz)");

    while (!stop && rclcpp::ok())
    {
        // ── Step 1: borrow iceoryx chunk for vision pipeline ──────────────────
        auto loaned_msg = pub->borrow_loaned_message();

        auto & msg = loaned_msg.get();

        // ── Step 2: Mat header over iceoryx shm ───────────────────────────────
        cv::Mat frame(IMG_HEIGHT, IMG_WIDTH, IMG_TYPE, msg.data_color.data());

        // ── Step 3: capture — V4L2 writes directly into iceoryx shm ──────────
        // simulates RealSense: color.get_data() → memcpy into iceoryx chunk
        // with virtual camera cap.read() writes directly so no memcpy here
        if (!cap.read(frame)) {
            RCLCPP_WARN(node->get_logger(), "Blank frame — skipping");
            continue;
        }

        // ── Step 4: shared capture timestamp ──────────────────────────────────
        int64_t capture_ns                = monotonic_now_ns();

        // ── Step 5: fill iceoryx message metadata ─────────────────────────────
        msg.image_intrinsics.width         = IMG_WIDTH;
        msg.image_intrinsics.height        = IMG_HEIGHT;
        msg.step_color                    = static_cast<uint32_t>(step);
        msg.is_bigendian                  = false;
        msg.frequency                     = CAM_FREQ_HZ;
        msg.timestamp                     = capture_ns;

        capture_ns                        = monotonic_now_ns();

        // ── Step 7: publish vision message (iceoryx zero-copy) ────────────────
        msg.publish_timestamp = capture_ns;

        pub->publish(std::move(loaned_msg));
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down dual publisher");
    cap.release();
    rclcpp::shutdown();
    return 0;
}