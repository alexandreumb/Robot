#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"

// ── native iceoryx headers ────────────────────────────────────────────────────
#include "iceoryx_posh/popo/untyped_subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"

#include <atomic>
#include <csignal>
#include <cstdint>
#include <time.h>
#include <sys/mman.h>

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

// ── service description — derived from rmw_iceoryx name conversion ────────────
static constexpr char IOX_SERVICE[]  = "fixed_size_msgs/msg/Image8Mb";
static constexpr char IOX_INSTANCE[] = "/camera_2";
static constexpr char IOX_EVENT[]    = "data";

// ── options ───────────────────────────────────────────────────────────────────
#define USE_CLOCK_MONOTONIC  1
#define USE_RT_SCHEDULING    0
#define USE_CPU_AFFINITY     0
#define USE_BUSY_WAIT        1   // 0 = WaitSet-like blocking, 1 = tight spin

constexpr int      SUBSCRIBER_CORE = 3;
constexpr int      RT_PRIORITY     = 80;
constexpr int      IMG_TYPE    = CV_16UC1;         

// ── globals ───────────────────────────────────────────────────────────────────
std::atomic<bool> stop{false};

void signal_handler(int) { stop = true; }

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

    // ── ROS2 init — still needed for logging and result publishing ────────────
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_publisher_node");

    configure_thread();
    
    // -- native untyped subscriber ---------------------------------------------
    // -- FastDDS publisher - publishes standard sensor_msgs/Image --------------    
    // Any FastDDS node (Nav2, Rviz, rosbag) can subscribe to this topic
    // QoS: reliable + volatile
    auto pub = node->create_publisher<sensor_msgs::msg::Image>(
        "/camera/depth",
        rclcpp::QoS(1).reliable().durability_volatile()
    );

    // -- Also publish the custom fixed_size message for any nodes that need it -
    auto pub_raw = node->create_publisher<Image8Mb>(
        "/camera_bridge",
        rclcpp::QoS(1).best_effort().durability_volatile()
    );
 
    RCLCPP_INFO(node->get_logger(),
        "Bridge started — iceoryx → FastDDS");
    RCLCPP_INFO(node->get_logger(),
        "  Input:  iceoryx %s / %s / %s",
        IOX_SERVICE, IOX_INSTANCE, IOX_EVENT);
    RCLCPP_INFO(node->get_logger(),
        "  Output: /camera/depth  (fixed_size_msgs/Image8Mb, FastDDS)");
    RCLCPP_INFO(node->get_logger(),
        "  Output: /camera_bridge (fixed_size_msgs/Image8Mb, FastDDS)");
 

    // -- native untyped subscriber ---------------------------------------------    
    // Must be called once per process. The runtime name must be unique.
    // This connects directly to RouDi shared memory - same RouDi that
    // rmw_iceoryx uses, so publisher and subscriber see the same segments.
    iox::runtime::PoshRuntime::initRuntime("camera_bridge_native");

    // -- native untyped subscriber ---------------------------------------------
    // UntypedSubscriber gives us a raw void* into shared memory — zero copy.
    // The service description must match exactly what rmw_iceoryx registered:
    //   service  = type_name  = "fixed_size_msgs/msg/Image8Mb"
    //   instance = topic_name = "/camera"
    //   event    = "data"     (always for ROS2 topics in rmw_iceoryx)
    iox::popo::UntypedSubscriber iox_sub(
        {IOX_SERVICE, IOX_INSTANCE, IOX_EVENT},
        []() {
            iox::popo::SubscriberOptions opts;
            opts.queueCapacity = 1;           // keep only latest — drop stale frames
            opts.historyRequest = 0;
            return opts;
        }()
    );

    uint64_t frame_count = 0;

    mlockall(MCL_CURRENT | MCL_FUTURE);
    // ── main loop ─────────────────────────────────────────────────────────────
    while (!stop && rclcpp::ok())
    {
#if USE_BUSY_WAIT
        // tight spin — zero wakeup latency, burns one CPU core
        iox_sub.take()
        .and_then([&](const void * userPayload) {

#else
        // blocking wait — yields CPU until data arrives
        // iceoryx WaitSet equivalent for untyped subscriber
        iox_sub.take()
        .and_then([&](const void * userPayload) {

#endif
            // -- Step 2: cast to message type — zero copy, no memcpy -----------
            // userPayload points directly into iceoryx shared memory segment.
            // The publisher wrote Image8Mb in-place via borrow_loaned_message.
            const auto * src = static_cast<const Image8Mb *>(userPayload);

            const int64_t time_ns = src->timestamp;
            // -- publish as sensor_msgs/Image for Nav2 / RViz ------------------
            // This involves a memcpy of the actual image data (height*step bytes)
            // but only copies the valid image region, not the full 12MB buffer.
            // At 640x480x3 = 921600 bytes this is ~46µs — acceptable for a bridge.
            auto ros_img = std::make_unique<sensor_msgs::msg::Image>();
            ros_img->header.stamp.sec = static_cast<int32_t>(time_ns / 1'000'000'000LL);
            ros_img->header.stamp.nanosec = static_cast<int32_t>(time_ns % 1'000'000'000LL);
            ros_img->header.frame_id = "camera_link";
            ros_img->height = src->height;
            ros_img->width = src->width;
            ros_img->encoding = "16UC1";
            ros_img->is_bigendian = src->is_bigendian;
            ros_img->step = src->step;

            // only copy the valid image data — not the full 12MB buffer
            const size_t data_size = src->height * src->step;
            ros_img->data.resize(data_size);
            std::memcpy(ros_img->data.data(), src->data.data(), data_size);

            pub->publish(std::move(ros_img));

            // -- also republish as fixed_size_msgs for any iceoryx-aware nodes -
            // This is a full 12MB copy — only use if downstream nodes need it
            // Comment out if not needed to save bandwidth
            // auto fixed_img = std::make_unique<Image8Mb>(*src);
            // pub_raw->publish(std::move(*fixed_img));

            iox_sub.release(userPayload);

            ++frame_count;            
            if (frame_count % 500 == 0)
            {
                RCLCPP_INFO(node->get_logger(), "Bridged %lu frames", frame_count);
            }
        })
        .or_else([](auto &) {
            // no chunk available — yield to avoid starving other threads
            __asm__ volatile("pause" ::: "memory");  // CPU hint to reduce power/contention
        });
    }
    RCLCPP_INFO(node->get_logger(), "Shutting down bridge");
    rclcpp::shutdown();
    return 0;
}