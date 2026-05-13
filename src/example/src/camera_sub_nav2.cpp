#include "rclcpp/rclcpp.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"

#include <atomic>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <time.h>
#include <vector>

#define USE_CLOCK_MONOTONIC 1
#define PRINT               0

static const std::string OUTPUT_DIR = std::string(getenv("HOME")) + "/latency_data";
constexpr size_t MAX_QUEUE_SIZE = 10;  // drop frames if disk can't keep up
constexpr int      IMG_TYPE    = CV_16UC1;         // bgr8, 2 bytes per pixel

std::atomic<bool> stop{false};
std::queue<std::pair<cv::Mat, uint64_t>> write_queue;  // frame + index
std::mutex queue_mutex;
std::condition_variable queue_cv;
std::atomic<bool> writer_stop{false};

void signal_handler(int) {stop = true;}

inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

void process_image(const cv::Mat & img) { (void)img; }

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
    const std::vector<double> & full_us)
{
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "[ERROR] Cannot open " << path << " for writing\n";
        return;
    }
    f << "frame,full_us\n";
    for (size_t i = 0; i < full_us.size(); ++i) {
        f << i << ","
          << std::fixed << std::setprecision(2) << full_us[i] << "\n";
    }
    std::cout << "[INFO] Saved " << full_us.size()
              << " frames to " << path << "\n";
}

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
    
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_subscriber_node");;

    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth",
        rclcpp::QoS(1).best_effort().durability_volatile(),
        [](const sensor_msgs::msg::Image::SharedPtr) {}
    );

    auto msg = std::make_unique<sensor_msgs::msg::Image>();

    std::vector<double> full_us_vec;
    std::vector<double> transport_us_vec;
    full_us_vec.reserve(10000);

    double   total_full_us      = 0.0;
    uint64_t frame_count        = 0;
    int      previous_id        = -1;
//    int      skipped            = 0;

    const std::string csv_path = make_csv_path();
    RCLCPP_INFO(node->get_logger(),
        "FastDDS subscriber started — saving to %s", csv_path.c_str());

    while (!stop && rclcpp::ok())
    {
        rclcpp::MessageInfo msg_info;
        if (!sub->take(*msg, msg_info)) {
            __asm__ volatile("pause" ::: "memory");
            continue;
        }


        ++frame_count;

        cv::Mat img(
            static_cast<int>(msg->height),
            static_cast<int>(msg->width),
            IMG_TYPE,
            const_cast<uint8_t *>(msg->data.data())
        );
        
        //process_image(img);
        cv::Mat depth_display;
        cv::normalize(img, depth_display, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        std::lock_guard<std::mutex> lock(queue_mutex);
        if (write_queue.size() < MAX_QUEUE_SIZE) {
            write_queue.push({img.clone(), frame_count});
            queue_cv.notify_one();
        }

#if USE_CLOCK_MONOTONIC
        const int64_t receive_ns = monotonic_now_ns();
#else
        const int64_t receive_ns = node->now().nanoseconds();
#endif

        const int64_t stamp_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1'000'000'000LL
                            + static_cast<int64_t>(msg->header.stamp.nanosec);

        const double full_us = static_cast<double>(receive_ns - stamp_ns) / 1000.0;


        full_us_vec.push_back(full_us);

        total_full_us+=full_us;

#if PRINT
        if (frame_count % 150 == 0) {
            RCLCPP_INFO(node->get_logger(),
                "Frames: %lu | full A→D: %.1f us | transport B→D: %.1f us",
                frame_count,
                total_full_us      / static_cast<double>(frame_count));
        }
#endif
    }

    save_csv(csv_path, full_us_vec);

    if (frame_count > 0) {
        const double avg_full      = total_full_us      / static_cast<double>(frame_count);
        RCLCPP_INFO(node->get_logger(), "──────────────────────────────────────────");
        RCLCPP_INFO(node->get_logger(), "Transport time summary");
        RCLCPP_INFO(node->get_logger(), "  Frames received      : %lu",   frame_count);
        RCLCPP_INFO(node->get_logger(), "  Full latency   A→D   : %.1f us (%.3f ms)", avg_full, avg_full/ 1000.0);
//        RCLCPP_INFO(node->get_logger(), "  Skipped frames       : %d",     skipped);
        RCLCPP_INFO(node->get_logger(), "  CSV saved to         : %s",     csv_path.c_str());
        RCLCPP_INFO(node->get_logger(), "──────────────────────────────────────────");
    } else {
        RCLCPP_WARN(node->get_logger(), "No frames received");
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down FastDDS subscriber");
    rclcpp::shutdown();
    return 0;
}