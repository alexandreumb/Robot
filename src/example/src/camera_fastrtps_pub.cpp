#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <librealsense2/rs.hpp>
#include "iceoryx_posh/popo/wait_set.hpp"

#include <atomic>
#include <csignal>
#include <cstdint>
#include <time.h>
#include <sys/mman.h>

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

#define USE_CLOCK_MONOTONIC 1
#define REALSENSE           0
#define USE_RT_SCHEDULING   1
#define USE_CPU_AFFINITY    1

std::atomic<bool> stop{false};
void signal_handler(int) { stop = true; }

inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

constexpr int      RT_PRIORITY = 40;     //needs to be inferior to the prority of the nvidia drivers which is 50
constexpr int      SUBSCRIBER_CORE = 2;

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
constexpr uint32_t CAM_FREQ_HZ = 30;

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
    std::cout << argv << std::endl;

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_publisher_node");

    // ── QoS: reliable works with FastDDS, no special constraints ─────────────
    auto qos = rclcpp::QoS(1)
        .reliable()
        .durability_volatile();

    configure_thread();
    mlockall(MCL_CURRENT | MCL_FUTURE);

#if REALSENSE
    auto pub = node->create_publisher<Image8Mb>("camera/color/image_raw", qos);

    rs2::pipeline p;
    rs2::config cfg;
    //The depth is meters is equal to depth scale * pixel value.
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    rs2::pipeline_profile profile = p.start(cfg);

    auto color_profile = profile.get_stream(RS2_STREAM_COLOR);
    rs2_intrinsics color_intrinsics = color_profile.as<rs2::video_stream_profile>().get_intrinsics();
    // get depth scale for metadata
    rs2::depth_sensor depth_sensor = p.get_active_profile()
        .get_device()
        .first<rs2::depth_sensor>();
    float depth_scale = depth_sensor.get_depth_scale();

#else
    auto pub = node->create_publisher<Image8Mb>("/camera/color/image_raw", qos);

    auto image = cv::imread("/home/alexandre/Pictures/images.jpg", cv::IMREAD_COLOR);
    if (image.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load image");
        return 1;
    }

    if (image.cols != IMG_WIDTH || image.rows != IMG_HEIGHT ||    image.type() != CV_8UC3)
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Unexpected image format: %dx%d type=%d",
            image.cols,
            image.rows,
            image.type()
        );
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

    // ── camera init ──────────────────────────────────────────────────────────
    /*
    cv::VideoCapture cap(CAM_INDEX, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open camera index %d", CAM_INDEX);
        return 1;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  IMG_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    */

#endif

    double time_dif = 0;
    int ite = 0;
    while (!stop && rclcpp::ok())
    {
        
#if REALSENSE
        if (pub->can_loan_messages())
        {
            RCLCPP_INFO_ONCE(node->get_logger(), "Publisher supports loaned messages — using zero-copy path");

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

            const size_t step_color = IMG_WIDTH * PIXEL_BYTES_COLOR;
            const size_t step_depth = IMG_WIDTH * PIXEL_BYTES_DEPTH;

            memcpy(msg.data_depth.data(), depth.get_data(), depth.get_height() * depth.get_stride_in_bytes());
            memcpy(msg.data_color.data(), color.get_data(), color.get_height() * color.get_stride_in_bytes());

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
            rs2::frameset frames = p.wait_for_frames();
            frames = align_to_color.process(frames);
            
            rs2::color_frame color = frames.get_color_frame();
            rs2::depth_frame depth = frames.get_depth_frame();
            
            auto msg = std::make_unique<Image8Mb>();
            
#if USE_CLOCK_MONOTONIC
            msg->timestamp = monotonic_now_ns();
#else
            msg->timestamp = node->now().nanoseconds();
#endif

            const size_t step_color = IMG_WIDTH * PIXEL_BYTES_COLOR;
            const size_t step_depth = IMG_WIDTH * PIXEL_BYTES_DEPTH;
            
            memcpy(msg->data_color.data(), color.get_data(), color.get_height() * color.get_stride_in_bytes());
            memcpy(msg->data_depth.data(), depth.get_data(), depth.get_height() * depth.get_stride_in_bytes());
    
            msg->image_intrinsics.width = IMG_WIDTH;
            msg->image_intrinsics.height = IMG_HEIGHT;
            msg->image_intrinsics.fx = color_intrinsics.fx;
            msg->image_intrinsics.fy = color_intrinsics.fy;
            msg->image_intrinsics.ppx = color_intrinsics.ppx;
            msg->image_intrinsics.ppy = color_intrinsics.ppy;
            msg->image_intrinsics.depth_units = depth_scale;
            msg->step_depth   = depth.get_stride_in_bytes();
            msg->step_color   = color.get_stride_in_bytes();
            msg->is_bigendian = false;
            msg->frequency    = CAM_FREQ_HZ;
                        
#if USE_CLOCK_MONOTONIC
            msg->publish_timestamp = monotonic_now_ns();
#else
            msg->publish_timestamp = node->now().nanoseconds();
#endif           
            pub->publish(*msg);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1000 / CAM_FREQ_HZ)
            );
        }

#else
        /*
        if (!cap.read(frame)) {
            RCLCPP_WARN(node->get_logger(), "Blank frame — skipping");
            // loaned_msg destructor returns chunk to pool automatically
            continue;
        }
        */
       
        const size_t step = IMG_WIDTH * PIXEL_BYTES;

        RCLCPP_INFO_ONCE(
            node->get_logger(),
            "RMW loan support: %s",
            pub->can_loan_messages() ? "YES" : "NO"
        );
        if (pub->can_loan_messages())
        {
            auto loaned_msg = pub->borrow_loaned_message();
            auto & msg = loaned_msg.get();

#if USE_CLOCK_MONOTONIC
            msg.timestamp = monotonic_now_ns();
#else
            msg.timestamp = node->now().nanoseconds();
#endif
            memcpy(msg.data_color.data(), image.data, IMG_WIDTH * IMG_HEIGHT * PIXEL_BYTES);
            if (ite > 500)
                time_dif += (double)((monotonic_now_ns() - msg.timestamp)/1e6);
            ite+= 1;
            msg.image_intrinsics.width        = IMG_WIDTH;
            msg.image_intrinsics.height       = IMG_HEIGHT;
            msg.step_color   = static_cast<uint32_t>(step);
            msg.is_bigendian = false;
            msg.frequency    = CAM_FREQ_HZ;
            
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
            
            auto msg = std::make_unique<Image8Mb>();

#if USE_CLOCK_MONOTONIC
            msg->timestamp = monotonic_now_ns();
#else
            msg->timestamp = node->now().nanoseconds();
#endif
            memcpy(msg->data_color.data(), image.data, IMG_WIDTH * IMG_HEIGHT * PIXEL_BYTES);
            msg->image_intrinsics.width        = IMG_WIDTH;
            msg->image_intrinsics.height       = IMG_HEIGHT;
            msg->step_color   = static_cast<uint32_t>(step);
            msg->is_bigendian = false;
            msg->frequency    = CAM_FREQ_HZ;
            
#if USE_CLOCK_MONOTONIC
            msg->publish_timestamp = monotonic_now_ns();
#else
            msg->publish_timestamp = node->now().nanoseconds();
#endif
            pub->publish(*msg);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1000 / CAM_FREQ_HZ)
            );
        }
#endif
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down FastDDS publisher");
    RCLCPP_INFO(node->get_logger(), "%f ms", (time_dif/(ite-500)));
#if REALSENSE
#else
    //cap.release();
#endif
    rclcpp::shutdown();
    return 0;
}
