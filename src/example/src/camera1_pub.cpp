#include "msgs/msg/image8_mb.hpp"
#include "iceoryx_posh/popo/untyped_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include <librealsense2/rs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <time.h>       // clock_gettime, CLOCK_MONOTONIC
#include <sys/mman.h>   // for mlockall 
#include <vector>

using Image8Mb = msgs::msg::Image8Mb;

// ── must match the subscriber setting ────────────────────────────────────────
// When USE_CLOCK_MONOTONIC = 1 in the subscriber, set it to 1 here too.
// Both sides must use the same clock or diffs will be meaningless.
#define USE_CLOCK_MONOTONIC 1
#define REALSENSE           1
#define USE_RT_SCHEDULING   1
#define USE_CPU_AFFINITY    1

static constexpr char IOX_SERVICE[]  = "msgs/msg/Image8Mb";
static constexpr char IOX_INSTANCE[] = "/camera";
static constexpr char IOX_EVENT[]    = "data";

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
constexpr int      RT_PRIORITY = 40;     //needs to be inferior to the prority of the nvidia drivers which is 50
constexpr int      SUBSCRIBER_CORE = 4;

constexpr int      CAM_INDEX   = 2;
constexpr int      IMG_WIDTH   = 565;
constexpr int      IMG_HEIGHT  = 489;
constexpr int      WARMUP_CHUNKS = 8; // >= your mempool count for this chunk size
constexpr int      IMG_TYPE_DEPTH    = CV_16UC1;      
constexpr int      IMG_TYPE_COLOR    = CV_8UC3;         // bgr8, 3 bytes per pixel
constexpr size_t   PIXEL_BYTES = 3;
constexpr uint32_t CAM_FREQ_HZ = 30;              // set to known camera frequency

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
    // ── signal setup ─────────────────────────────────────────────────────────
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── ROS2 init — still needed for logging ─────────────────────────────────
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_publisher_node");

    iox::runtime::PoshRuntime::initRuntime("camera_publisher_native");
    iox::popo::UntypedPublisher pub({IOX_SERVICE, IOX_INSTANCE, IOX_EVENT});
    
    configure_thread();
    mlockall(MCL_CURRENT | MCL_FUTURE);

#if REALSENSE
    rs2::pipeline p;
    rs2::config cfg;
    //The depth is meters is equal to depth scale * pixel value.
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    rs2::pipeline_profile profile = p.start(cfg);

    auto color_profile = profile.get_stream(RS2_STREAM_COLOR);
    rs2_intrinsics color_intrinsics = color_profile.as<rs2::video_stream_profile>().get_intrinsics();
    // get depth scale for metadata
    rs2::depth_sensor depth_sensor = p.get_active_profile()
        .get_device()
        .first<rs2::depth_sensor>();
    float depth_scale = depth_sensor.get_depth_scale();

#else

    auto image = cv::imread("/home/robotics4farmers/Pictures/Screenshots/r4f_icon.png", cv::IMREAD_COLOR);

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

    const size_t step       = IMG_WIDTH * PIXEL_BYTES;
    const size_t frame_size = step * IMG_HEIGHT;


#endif
    std::vector<void*> warmup_chunks;
    warmup_chunks.reserve(WARMUP_CHUNKS);

    for (int i = 0; i < WARMUP_CHUNKS; ++i) {
        pub.loan(sizeof(Image8Mb))
            .and_then([&](void* userPayload) {
                std::memset(userPayload, 0, sizeof(Image8Mb)); // force every page to fault in now
                warmup_chunks.push_back(userPayload);
            })
            .or_else([](auto& error) {
                std::cerr << "Warm-up loan failed: " << error << std::endl;
            });
    }

    for (void* chunk : warmup_chunks) {
        pub.release(chunk);
    }

    rs2::align align_to_color(RS2_STREAM_COLOR);
    double time_dif = 0;
    int ite = 0;
    // ── main capture + publish loop ───────────────────────────────────────────
    while (!stop)
    {
        RCLCPP_INFO_ONCE(node->get_logger(), "Publishing at %d Hz", CAM_FREQ_HZ);
        
#if REALSENSE
        rs2::frameset frames   = p.wait_for_frames();
        frames = align_to_color.process(frames);

        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::video_frame color = frames.get_color_frame();
#endif
        // ── Step 1: borrow a loaned chunk from iceoryx ────────────────────────
        pub.loan(sizeof(Image8Mb))
            .and_then([&](void *userPayload) {
                auto *msg = new (userPayload) Image8Mb;

#if USE_CLOCK_MONOTONIC
                auto timestamp = monotonic_now_ns();
                msg->header.stamp.sec = timestamp / 1'000'000'000LL;
                msg->header.stamp.nanosec = timestamp % 1'000'000'000LL;
#else
                msg->header.stamp    = node->now();
#endif                

#if REALSENSE
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

#else
                memcpy(msg->data_color.data(), image.data, IMG_WIDTH * IMG_HEIGHT * PIXEL_BYTES);
                msg->step_color         = static_cast<uint32_t>(step);
                msg->image_intrinsics.width = IMG_WIDTH;
                msg->image_intrinsics.height = IMG_HEIGHT;
#endif
            
                time_dif += (double)((monotonic_now_ns() - timestamp)/1e6);
                ite+= 1;
                msg->is_bigendian = false;
                msg->frequency    = CAM_FREQ_HZ;

#if USE_CLOCK_MONOTONIC
                msg->publish_timestamp = monotonic_now_ns();
#else
                msg->publish_timestamp = node->now().nanoseconds();
#endif
                pub.publish(userPayload);
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(1000 / CAM_FREQ_HZ)
                );
            })
            .or_else([](auto& error) {
                // Do something with error
                std::cerr << "Unable to loan sample, error: " << error << std::endl;
            });
    }
    RCLCPP_INFO(node->get_logger(), "%f ms", (time_dif/ite));

#if REALSENSE
#else
    /*
    cap.release();
    */
#endif
    return 0;
}
