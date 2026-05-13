#include "fixed_size_msgs/msg/image8_mb.hpp"
#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <time.h>       // clock_gettime, CLOCK_MONOTONIC
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <limits>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

static const std::string OUTPUT_DIR = std::string(getenv("HOME")) + "/latency_data";


// ── options ───────────────────────────────────────────────────────────────────
#define USE_CLOCK_MONOTONIC  1
#define USE_RT_SCHEDULING    0
#define USE_CPU_AFFINITY     0
#define USE_BUSY_WAIT        1   // 0 = WaitSet-like blocking, 1 = tight spin

constexpr size_t MAX_QUEUE_SIZE = 10;  // drop frames if disk can't keep up
constexpr int      SUBSCRIBER_CORE = 3;
constexpr int      RT_PRIORITY     = 80;
constexpr int      IMG_TYPE    = CV_8UC3;         // bgr8, 3 bytes per pixel

// ── globals ───────────────────────────────────────────────────────────────────
std::atomic<bool> stop{false};
std::queue<std::pair<cv::Mat, uint64_t>> write_queue;  // frame + index
std::mutex queue_mutex;
std::condition_variable queue_cv;
std::atomic<bool> writer_stop{false};

static double   total_full_us     = 0.0;
static double   total_transport_us = 0.0;

void signal_handler(int) { stop = true; }

// ── helpers ───────────────────────────────────────────────────────────────────
inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

std::string make_csv_path()
{
    system(("mkdir -p " + OUTPUT_DIR).c_str());
    time_t now = time(nullptr);
    struct tm * t = localtime(&now);
    std::ostringstream ss;
    ss << OUTPUT_DIR << "/latency_run_"
       << std::setfill('0')
       << (t->tm_year + 1900) << "-"
       << std::setw(2) << (t->tm_mon + 1) << "-"
       << std::setw(2) << t->tm_mday << "_"
       << std::setw(2) << t->tm_hour << "-"
       << std::setw(2) << t->tm_min  << "-"
       << std::setw(2) << t->tm_sec
       << ".csv";
    return ss.str();
}

void save_csv(
    const std::string & path,
    const std::vector<double> & full_us,
    const std::vector<double> & transport_us)
{
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "[ERROR] Cannot open " << path << " for writing\n";
        return;
    }
    f << "frame,full_us,transport_us\n";
    for (size_t i = 0; i < full_us.size(); ++i) {
        f << i << ","
          << std::fixed << std::setprecision(2) << full_us[i] << ","
          << std::fixed << std::setprecision(2) << transport_us[i] << "\n";
    }
    std::cout << "[INFO] Saved " << full_us.size()
              << " frames to " << path << "\n";
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

// ── placeholder pipeline ──────────────────────────────────────────────────────
void process_image(const cv::Mat & img) { (void)img; }

int main(int argc, char ** argv)
{
    std::thread writer_thread([&]() {
    while (!writer_stop || !write_queue.empty()) {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [&]{ 
            return !write_queue.empty() || writer_stop; 
        });
        if (write_queue.empty()) continue;
        auto [frame, idx] = write_queue.front();
        write_queue.pop();
        lock.unlock();
        // write outside lock — doesn't block receive loop
        cv::imwrite("image" + std::to_string(idx) + ".png", frame);
    }
    });
    
    // ── signal setup ─────────────────────────────────────────────────────────
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── ROS2 init ────────────────────────────────────────────────────────────
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_subscriber_node");

    configure_thread();

    // ── QoS: must be best_effort + volatile for iceoryx zero-copy ────────────
    auto qos = rclcpp::QoS(1)
        .best_effort()
        .durability_volatile();

    mlockall(MCL_CURRENT | MCL_FUTURE);
    auto sub = node->create_subscription<Image8Mb>("camera", qos, [](Image8Mb msg) {} );

    std::vector<double> full_us_vec;
    std::vector<double> transport_us_vec;
    full_us_vec.reserve(10000);
    transport_us_vec.reserve(10000);

    double   min_transport_us   = std::numeric_limits<double>::max();
    double   max_transport_us   = 0.0;
    uint64_t frame_count        = 0;
    int      previous_id        = -1;

    auto msg = std::make_unique<fixed_size_msgs::msg::Image8Mb>();

    rclcpp::MessageInfo info;

    const std::string csv_path = make_csv_path();
    RCLCPP_INFO(node->get_logger(),
        "Ros2+Iceoryx subscriber started — saving to %s", csv_path.c_str());

    mlockall(MCL_CURRENT | MCL_FUTURE);

    // ── main loop ─────────────────────────────────────────────────────────────
    while (!stop && rclcpp::ok())
    {
        // ── Step 1: check if we there is any message to get ────────────────────────
        try {
            if (!sub->take(*msg, info)) {
                __asm__ volatile("pause" ::: "memory");
                continue;
            }
        }
        catch (const rclcpp::exceptions::RCLError & e) {
            // rmw_iceoryx throws when queue is empty instead of returning false
            // this is a known bug in rmw_iceoryx — just continue spinning
            __asm__ volatile("pause" ::: "memory");
            continue;
        }

        // ── Step 2: timestamp receive ─────────────────────────────────────────────
        // Timestamp sampled immediately after read() — closest to capture time.
        // Must use the same clock as the subscriber — both USE_CLOCK_MONOTONIC
        // must be s    et to the same value or latency diffs will be wrong.
#if USE_CLOCK_MONOTONIC
        const int64_t receive_ns = monotonic_now_ns();
#else
        const int64_t receive_ns = node->now().nanoseconds();
#endif
        
        // ── Step 3: measure latency ───────────────────────────────────────
        const double full_us = static_cast<double>(
            receive_ns - msg->timestamp) / 1000.0;
        const double transport_us = static_cast<double>(
            receive_ns - msg->publish_timestamp) / 1000.0;
        
        previous_id = static_cast<int>(msg->publish_timestamp);

        full_us_vec.push_back(full_us);
        transport_us_vec.push_back(transport_us);  
        
        total_full_us      += full_us;
        total_transport_us += transport_us;
        min_transport_us    = std::min(min_transport_us, transport_us);
        max_transport_us    = std::max(max_transport_us, transport_us);
        ++frame_count;   
        
        // ── Step 4: construct Mat header over shared memory ───────────────
        cv::Mat img(
            static_cast<int>(msg->height),
            static_cast<int>(msg->width),
            IMG_TYPE,
            const_cast<uint8_t *>(msg->data.data())
        );

        // ── Step 5: process ───────────────────────────────────────────────
        //process_image(img);

        std::lock_guard<std::mutex> lock(queue_mutex);
        if (write_queue.size() < MAX_QUEUE_SIZE) {
            write_queue.push({img.clone(), frame_count});
            queue_cv.notify_one();
        }

    }
    
    writer_stop = true;
    queue_cv.notify_one();
    writer_thread.join();
    save_csv(csv_path, full_us_vec, transport_us_vec);

    // ── shutdown summary ──────────────────────────────────────────────────────
    if (frame_count > 0) {
        const double avg_full      = total_full_us      / static_cast<double>(frame_count);
        const double avg_transport = total_transport_us / static_cast<double>(frame_count);
        RCLCPP_INFO(node->get_logger(), "──────────────────────────────────────────");
        RCLCPP_INFO(node->get_logger(), "Transport time summary");
        RCLCPP_INFO(node->get_logger(), "  Frames received      : %lu",   frame_count);
        RCLCPP_INFO(node->get_logger(), "  Full latency   A→D   : %.1f us (%.3f ms)", avg_full,      avg_full      / 1000.0);
        RCLCPP_INFO(node->get_logger(), "  Transport only B→D   : %.1f us (%.3f ms)", avg_transport, avg_transport / 1000.0);
        RCLCPP_INFO(node->get_logger(), "  Transport min B→D    : %.1f us", min_transport_us);
        RCLCPP_INFO(node->get_logger(), "  Transport max B→D    : %.1f us", max_transport_us);
//        RCLCPP_INFO(node->get_logger(), "  Skipped frames       : %d",     skipped);
        RCLCPP_INFO(node->get_logger(), "  CSV saved to         : %s",     csv_path.c_str());
        RCLCPP_INFO(node->get_logger(), "──────────────────────────────────────────");
    } else {
        RCLCPP_WARN(node->get_logger(), "No frames received");
    }
    
    RCLCPP_INFO(node->get_logger(), "Shutting down ros2 + iceoryx subscriber");
    rclcpp::shutdown();
    return 0;
}