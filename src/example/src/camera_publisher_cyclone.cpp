#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/opencv.hpp>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <time.h>
#include <thread>
#include <chrono>
#include <cstring>
#include <sys/mman.h>   // for mlockall 

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

#define USE_CLOCK_MONOTONIC 1
#define REALSENSE           0
#define USE_RT_SCHEDULING   1
#define USE_CPU_AFFINITY    1

constexpr int      CAM_INDEX   = 2;
constexpr int      RT_PRIORITY = 40;     //needs to be inferior to the prority of the nvidia drivers which is 50
constexpr int      SUBSCRIBER_CORE = 2;
constexpr int      IMG_WIDTH   = 1280;
constexpr int      IMG_HEIGHT  = 720;
constexpr int      IMG_TYPE_DEPTH    = CV_16UC1;      
constexpr int      IMG_TYPE_COLOR    = CV_8UC3;         // bgr8, 3 bytes per pixel
constexpr int      IMG_TYPE    = CV_8UC3;         // bgr8, 3 bytes per pixel
constexpr size_t   PIXEL_BYTES_DEPTH = 2;
constexpr size_t   PIXEL_BYTES_COLOR = 3;// bgr8, 3 bytes per pixel
constexpr size_t   PIXEL_BYTES = 3;
constexpr uint32_t CAM_FREQ_HZ = 30;


std::atomic<bool> stop{false};

void signal_handler(int) { stop = true; }

inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

void configure_thread()
{
#if USE_RT_SCHEDULING
    struct sched_param param{};
    param.sched_priority = RT_PRIORITY;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        std::cerr << "[WARN] Failed to set RT scheduling\n";
    } else {
        std::cout << "[INFO] RT scheduling set: SCHED_FIFO priority " << RT_PRIORITY << "\n";
    }
#endif
#if USE_CPU_AFFINITY
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);  
    CPU_SET(SUBSCRIBER_CORE, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
        std::cerr << "[WARN] Failed to set CPU affinity\n";
    } else {
        std::cout << "[INFO] Thread pinned to core " << SUBSCRIBER_CORE << "\n";
    }
#endif
}

int main(int argc, char ** argv)
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_publisher_node");

    // ── QoS: best_effort + volatile required for zero-copy loaned messages ────
    // CycloneDDS shared memory works with both reliable and best_effort,
    // but loaned messages require volatile durability.
    auto qos = rclcpp::QoS(
        rclcpp::KeepLast(1)
    )
    .reliable()
    .durability_volatile();

    configure_thread();
    mlockall(MCL_CURRENT | MCL_FUTURE);

#if REALSENSE
    auto pub = node->create_publisher<Image8Mb>("camera/color/image_raw", qos);
    
    rs2::pipeline p;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_BGR8, CAM_FREQ_HZ);
    cfg.enable_stream(RS2_STREAM_DEPTH, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_Z16, CAM_FREQ_HZ);
    rs2::pipeline_profile profile = p.start(cfg);

    auto color_profile = profile.get_stream(RS2_STREAM_COLOR);
    rs2_intrinsics color_intrinsics = color_profile.as<rs2::video_stream_profile>().get_intrinsics();
    // get depth scale for metadata
    rs2::depth_sensor depth_sensor = p.get_active_profile()
        .get_device()
        .first<rs2::depth_sensor>();
    float depth_scale = depth_sensor.get_depth_scale();

#else
    auto pub = node->create_publisher<Image8Mb>("camera/color/image_raw", qos);

    auto image = cv::imread("/home/alexandre/Pictures/images.jpg", cv::IMREAD_COLOR);
    if (image.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load image");
        return 1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Image: %dx%d, channels=%d, step=%zu",
        image.cols,
        image.rows,
        image.channels(),
        image.step
    );
    /*
    cv::VideoCapture cap(CAM_INDEX, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open camera index %d", CAM_INDEX);
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  IMG_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
    cap.set(cv::CAP_PROP_BUFFERSIZE,   1);
    */
#endif

    RCLCPP_INFO(node->get_logger(), "Camera publisher started");
    while (!stop && rclcpp::ok())
    {
        RCLCPP_INFO_ONCE(
            node->get_logger(),
            "can_loan_messages() = %s",
            pub->can_loan_messages() ? "true" : "false"
        );
        if (pub->can_loan_messages())
        {
            RCLCPP_INFO_ONCE(node->get_logger(), "Publisher supports loaned messages — using zero-copy path");


#if REALSENSE
            rs2::frameset frames = p.wait_for_frames();
            frames = align_to_color.process(frames);

            rs2::color_frame color = frames.get_color_frame();
            rs2::depth_frame depth = frames.get_depth_frame();


            auto loaned_msg = pub->borrow_loaned_message();
            auto & msg      = loaned_msg.get();
#if USE_CLOCK_MONOTONIC
            msg.timestamp = monotonic_now_ns();
#else
            msg.timestamp = node->now().nanoseconds();
#endif            
            cv::Mat depth_frame(
                IMG_HEIGHT,
                IMG_WIDTH,
                IMG_TYPE_DEPTH,
                msg.data_depth.data()
            );

            cv::Mat color_frame(
                IMG_HEIGHT,
                IMG_WIDTH,
                IMG_TYPE_COLOR,
                msg.data_color.data()
            );

            const size_t step_color = IMG_WIDTH * PIXEL_BYTES_COLOR;
            const size_t step_depth = IMG_WIDTH * PIXEL_BYTES_DEPTH;

            memcpy(depth_frame.data, depth.get_data(), depth.get_height() * depth.get_stride_in_bytes());
            memcpy(color_frame.data, color.get_data(), color.get_height() * color.get_stride_in

            msg.image_intrinsics.width = IMG_WIDTH;
            msg.image_intrinsics.height = IMG_HEIGHT;
            msg.image_intrinsics.fx = color_intrinsics.fx;
            msg.image_intrinsics.fy = color_intrinsics.fy;
            msg.image_intrinsics.ppx = color_intrinsics.ppx;
            msg.image_intrinsics.ppy = color_intrinsics.ppy;
            msg.image_intrinsics.depth_units = depth_scale;
            msg.step_depth   = depth.get_stride_in_bytes();
            msg.step_color   = color.get_stride_in_bytes();
            msg.is_bigendian = false;
            msg.frequency    = CAM_FREQ_HZ;
#else
            auto loaned_msg = pub->borrow_loaned_message();
            auto & msg      = loaned_msg.get();
#if USE_CLOCK_MONOTONIC
            msg.timestamp = monotonic_now_ns();
#else
            msg.timestamp = node->now().nanoseconds();
#endif      
            cv::Mat depth_frame(
                IMG_HEIGHT,
                IMG_WIDTH,
                IMG_TYPE_DEPTH,
                msg.data_depth.data()
            );

            cv::Mat color_frame(
                IMG_HEIGHT,
                IMG_WIDTH,
                IMG_TYPE_COLOR,
                msg.data_color.data()
            );

            /*
            if (!cap.read(color_frame)) {
                RCLCPP_WARN(node->get_logger(), "Blank frame — skipping");
                continue;
            }
            */
            memcpy(color_frame.data, image.data, IMG_WIDTH * IMG_HEIGHT * PIXEL_BYTES);
            const size_t step = IMG_WIDTH * PIXEL_BYTES;

            msg.image_intrinsics.width = IMG_WIDTH;
            msg.image_intrinsics.height = IMG_HEIGHT;
            msg.step_color         = static_cast<uint32_t>(step);
            msg.is_bigendian = false;
            msg.frequency    = CAM_FREQ_HZ;

#endif

#if USE_CLOCK_MONOTONIC
            msg.publish_timestamp = monotonic_now_ns();
#else
            msg.publish_timestamp = node->now().nanoseconds();
#endif
            pub->publish(std::move(loaned_msg));
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1000 / CAM_FREQ_HZ)
            );
        }
        else
        {
            auto msg_ptr = std::make_unique<Image8Mb>();
            Image8Mb & msg = *msg_ptr;
            cv::Mat frame(IMG_HEIGHT, IMG_WIDTH, IMG_TYPE, msg.data_color.data());
            /*
            if (!cap.read(frame)) {
                RCLCPP_WARN(node->get_logger(), "Blank frame — skipping");
                continue;
            }
            */

#if USE_CLOCK_MONOTONIC
            msg.timestamp = monotonic_now_ns();
#else
            msg.timestamp = node->now().nanoseconds();
#endif
            const size_t step = IMG_WIDTH * PIXEL_BYTES;

            msg.image_intrinsics.width        = IMG_WIDTH;
            msg.image_intrinsics.height       = IMG_HEIGHT;
            msg.step_color         = static_cast<uint32_t>(step);
            msg.is_bigendian = false;
            msg.frequency    = CAM_FREQ_HZ;

#if USE_CLOCK_MONOTONIC
            msg.publish_timestamp = monotonic_now_ns();
#else
            msg.publish_timestamp = node->now().nanoseconds();
#endif
            pub->publish(msg);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down camera publisher");
    //cap.release();
    rclcpp::shutdown();
    return 0;
}
