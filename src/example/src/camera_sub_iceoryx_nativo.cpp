#include "msgs/msg/image8_mb.hpp"
#include "msgs/msg/img_analyze_msg.hpp"
#include <opencv4/opencv2/opencv.hpp>

// ── native iceoryx headers ────────────────────────────────────────────────────
#include "iceoryx_posh/popo/untyped_subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "Shared_Memory_Sensors/header_accessor.h"
#include "iceoryx_hoofs/cxx/optional.hpp"
#include "iceoryx_hoofs/posix_wrapper/signal_handler.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx_posh/popo/user_trigger.hpp"
#include "iceoryx_posh/popo/wait_set.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"

#define GPU 1
#if GPU
    #include "Yolo_Tensorrt/yolov8.h"
#endif

#include <atomic>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <limits>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <condition_variable>
#include <optional>

using Image8Mb = msgs::msg::Image8Mb;
using ImgAnalyze = msgs::msg::ImgAnalyzeMsg;
iox::cxx::optional<iox::popo::WaitSet<>> waitset;

// ── service description — derived from rmw_iceoryx name conversion ────────────
// rule: {type_name, topic_name, "data"} for all ROS2 topics
// type_name  = package/msg/Type  → "fixed_size_msgs/msg/Image8Mb"
// topic_name = /camera
// event      = "data"            (always for ROS2)
static constexpr char IOX_SERVICE[]  = "msgs/msg/Image8Mb";
static constexpr char IOX_INSTANCE[] = "/camera";
static constexpr char IOX_EVENT[]    = "data";
static const std::string OUTPUT_DIR = std::string(getenv("HOME")) + "/latency_data/";

// ── options ───────────────────────────────────────────────────────────────────
#define USE_CLOCK_MONOTONIC  1
#define USE_RT_SCHEDULING    0
#define USE_CPU_AFFINITY     1
#define REALSENSE            1
#define SAVE_CSV             1
#define PUBLISH_METHOD       1 // 1 = publish with thread

constexpr int      SUBSCRIBER_CORE = 2;
constexpr int      RT_PRIORITY     = 40;
constexpr int      IMG_TYPE_COLOR    = CV_8UC3;         // bgr8, 3 bytes per pixel
constexpr int      IMG_TYPE_DEPTH    = CV_16UC1;         

// ── struct ──────────────────────────────────────────────────────
struct Frame
{
    cv::Mat color;
    cv::Mat depth;
    image_intrinsics intrinsics;
    int64_t timestamp;
    int64_t transfer_time;
};

struct TimesToAnalyze
{
    double full_ms;
    double transport_ms;
    double process_time_ms;
    double transfer_thread_ms;
};  

// ── globals ───────────────────────────────────────────────────────────────────
std::atomic<bool> stop{false};
std::mutex write_mutex;
std::optional<Frame> latest_frame;
std::optional<TimesToAnalyze> latest_times;
std::condition_variable frame_cv;

static double   total_full_ms{0.0};
static double   total_transport_ms{0.0};

// ── helpers ───────────────────────────────────────────────────────────────────
void signal_handler(int sig IOX_MAYBE_UNUSED) { 
    stop = true; 

    if (waitset)
        waitset->markForDestruction();
}

inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

std::string make_csv_path(std::string engine_file_path)
{
    std::string model;
    if (engine_file_path.find('11') != std::string::npos) {
        model = "yolo_rt";
    }
    else {
        model = "yolo8";
    }
    
    system(("mkdir -p " + OUTPUT_DIR + model).c_str());
    time_t now = time(nullptr);
    struct tm * t = localtime(&now);
    std::ostringstream ss;
    ss << OUTPUT_DIR << model
       << "/latency_run_"
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
    const std::vector<TimesToAnalyze> & times,
    const std::vector<int> & object_identified)
{
    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "[ERROR] Cannot open " << path << " for writing\n";
        return;
    }
    f << "frame,full_ms,transport_ms,process_ms,transfer_ms,object_identified\n";
    for (size_t i = 0; i < times.size(); ++i) {
        f << i << ","
          << std::fixed << std::setprecision(2) << times[i].full_ms << ","
          << std::fixed << std::setprecision(2) << times[i].transport_ms << ","
          << std::fixed << std::setprecision(2) << times[i].process_time_ms << ","
          << std::fixed << std::setprecision(2) << times[i].transfer_thread_ms << ","
          << object_identified[i] << "\n";
    }
    std::cout << "[INFO] Saved " << times.size()
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
#if GPU
std::vector<Object> process_image(YoloV8& detector, const cv::Mat& img_color, const cv::Mat& img_depth, image_intrinsics img_intrinsics)
{
    auto max_detected = 0.2f;
    std::vector<Object> objects = detector.detectObjects(img_color);
    auto ret = detector.extractObjects(img_depth, objects, img_intrinsics, max_detected);

    return ret;
}
#endif

int main(int argc, char ** argv)
{

// register signal handler to handle termination of the loop
auto signalGuard = iox::posix::registerSignalHandler(iox::posix::Signal::INT, signal_handler);
auto signalTermGuard = iox::posix::registerSignalHandler(iox::posix::Signal::TERM, signal_handler);

#if defined(__aarch64__) || defined(_M_ARM64)
    const std::string engine_file_path = "/home/robotics4farmers/Dev/Robot/src/example/Yolo_Tensorrt/r4f_yolo11l_seg.onnx";
#elif defined(__x86_64__) || defined(_M_X64)
    const std::string engine_file_path = "/home/robotics4farmers/Dev/Robot/src/example/Yolo_Tensorrt/r4f_yolo11l_seg.onnx";
#endif

#if GPU 
    YoloV8Config config;
    YoloV8 detector(engine_file_path, config);   
#endif

    // ── ROS2 init — still needed for logging and result publishing ────────────
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_subscriber_node");

    auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
    auto pub = node->create_publisher<msgs::msg::ImgAnalyzeMsg>(
        "robot_steering_controller/reference", qos);

    configure_thread();    
    // ── native iceoryx runtime init ───────────────────────────────────────────
    // Must be called once per process. The runtime name must be unique.
    // This connects directly to RouDi shared memory — same RouDi that
    // rmw_iceoryx uses, so publisher and subscriber see the same segments.
    iox::runtime::PoshRuntime::initRuntime("camera_subscriber_native");

    waitset.emplace();
    // ── native untyped subscriber ─────────────────────────────────────────────
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

#if SAVE_CSV
    std::vector<TimesToAnalyze> times;
    std::vector<int> object_identified;
    times.reserve(10000);
    object_identified.reserve(10000);
    int test{0};

#endif 

    double   min_transport_ms   = std::numeric_limits<double>::max();
    double   max_transport_ms   = 0.0;
    uint64_t frame_count        = 0;

    const std::string csv_path = make_csv_path(engine_file_path);
    mlockall(MCL_CURRENT | MCL_FUTURE);

// Processing thread
#if PUBLISH_METHOD
    std::thread process_image_and_pub([&] ()
    {
        int policy;
        sched_param param;

        pthread_getschedparam(pthread_self(), &policy, &param);

        std::cout << "policy = " << policy
                << ", priority = " << param.sched_priority
                << '\n';
        while (!stop)
        {
            Frame current_frame;
            TimesToAnalyze current_times;
            {
                std::unique_lock<std::mutex> lock(write_mutex);
                frame_cv.wait(lock, [&]{ return latest_frame.has_value() || stop;});

                if (stop)
                    break;

                current_frame = std::move(*latest_frame);
                current_times = std::move(*latest_times);
                latest_frame.reset();
                latest_times.reset();
            }
        
            auto transfer_duration = node->now().nanoseconds() - current_frame.transfer_time;
            ImgAnalyze obj_msg;
            obj_msg.has_object = 0;
#if GPU 
            auto before = node->now().nanoseconds();
            std::vector<Object> objects = process_image(detector, current_frame.color, current_frame.depth, current_frame.intrinsics);
            auto after = node->now().nanoseconds();
            auto process_duration = after - before;

#if SAVE_CSV
            current_times.transfer_thread_ms = (static_cast<double>(transfer_duration) / 1'000'000LL); // Convert to seconds
            current_times.process_time_ms = (static_cast<double>(process_duration) / 1'000'000LL); // Convert to milliseconds
            times.push_back(current_times);
            object_identified.push_back(objects.size());
#endif
            //RCLCPP_INFO(node->get_logger(), "Processing time: %.4f ms", process_duration / 1'000'000LL);

            /*
            if (objects.size() > 0)
            {   
                test += 1;
                //RCLCPP_INFO(node->get_logger(), "Frame %d.", test);
                //RCLCPP_INFO(node->get_logger(), "%lu objects detected in the current frame.", objects.size());
                obj_msg.has_object = 1;
                obj_msg.object.resize(objects.size());
                for (size_t i = 0; i < objects.size() && i < obj_msg.object.size(); ++i)
                {
                    obj_msg.object[i].label = objects[i].label;
                    obj_msg.object[i].probability = objects[i].probability;
                    obj_msg.object[i].box.x = objects[i].rect.x;
                    obj_msg.object[i].box.y = objects[i].rect.y;
                    obj_msg.object[i].box.width = objects[i].rect.width;
                    obj_msg.object[i].box.height = objects[i].rect.height;
                    obj_msg.object[i].kps = objects[i].kps;
                    obj_msg.object[i].point.pose_x = objects[i].Pose2D.x;
                    obj_msg.object[i].point.pose_y = objects[i].Pose2D.y;
                    obj_msg.object[i].point3d.x = objects[i].Pose3D[0];
                    obj_msg.object[i].point3d.y = objects[i].Pose3D[1];
                    obj_msg.object[i].point3d.z = objects[i].Pose3D[2];
                }
            }                 
            */
            
                  
#endif
            obj_msg.object.resize(1);
            obj_msg.object[0].label = 0;
            obj_msg.object[0].probability = 0.9;
            obj_msg.object[0].box.x = 100;
            obj_msg.object[0].box.y = 0;
            obj_msg.object[0].box.width = 400;
            obj_msg.object[0].box.height = 200;
            obj_msg.object[0].kps = {0.1,2.7,2.5,3.5};
            obj_msg.object[0].point.pose_x = 15;
            obj_msg.object[0].point.pose_y = 6;
            obj_msg.object[0].point3d.x = 10;
            obj_msg.object[0].point3d.y = 6;
            obj_msg.object[0].point3d.z = 1.2;
            obj_msg.header.stamp.sec = current_frame.timestamp / 1'000'000'000LL;
            obj_msg.header.stamp.nanosec = current_frame.timestamp % 1'000'000'000LL;
            obj_msg.middle_header.stamp = node->now();
            //RCLCPP_INFO(node->get_logger(), "Publishing image time: %llu.", (node->now().nanoseconds() - obj_msg.header.stamp.sec * 1'000'000'000LL - obj_msg.header.stamp.nanosec)%1'000'000'000LL);

            pub->publish(obj_msg);
        }
    });
#endif

    waitset->attachState(iox_sub, iox::popo::SubscriberState::HAS_DATA).or_else([](auto) {
        std::cerr << "failed to attach subscriber" << std::endl;
        std::exit(EXIT_FAILURE);
    });


    // ── main loop ─────────────────────────────────────────────────────────────
    while (!stop && rclcpp::ok())
    {
        auto notifications = waitset->wait();

        for (auto& notification: notifications) {
            if (notification->doesOriginateFrom(&iox_sub)) {
                iox_sub.take()
                .and_then([&](const void * userPayload) {
                    // ── Step 1: timestamp receive — userPayload is direct shm pointer ─
#if USE_CLOCK_MONOTONIC
                    const int64_t receive_ns = monotonic_now_ns();
#else
                    const int64_t receive_ns = node->now().nanoseconds();
#endif

                    const auto * msg = static_cast<const Image8Mb *>(userPayload);
                    // ── Step 3: measure latency ───────────────────────────────────────
                    const double full_ms = static_cast<double>(
                        receive_ns - msg->timestamp) / 1e6; // convert to milliseconds
                    const double transport_ms = static_cast<double>(
                        receive_ns - msg->publish_timestamp) / 1e6; // convert to miliseconds

                    total_full_ms      += full_ms;
                    total_transport_ms += transport_ms;
                    min_transport_ms    = std::min(min_transport_ms, transport_ms);
                    max_transport_ms    = std::max(max_transport_ms, transport_ms);
                    ++frame_count;

                    // ── Step 4: construct Mat header over shared memory ───────────────
                    // NO memcpy — Mat points directly at iceoryx chunk.
                    // process_image() must complete before release() is called below.
                    cv::Mat img_color(
                        static_cast<int>(msg->image_intrinsics.height),
                        static_cast<int>(msg->image_intrinsics.width),
                        IMG_TYPE_COLOR,
                        const_cast<uint8_t *>(msg->data_color.data())
                    );

                    cv::Mat img_depth(
                        static_cast<int>(msg->image_intrinsics.height),
                        static_cast<int>(msg->image_intrinsics.width),
                        IMG_TYPE_DEPTH,
                        const_cast<uint8_t *>(msg->data_depth.data())
                    );


                    image_intrinsics img_intrinsics;
                    img_intrinsics.width = msg->image_intrinsics.width;  
                    img_intrinsics.height = msg->image_intrinsics.height;
                    img_intrinsics.fx = msg->image_intrinsics.fx;
                    img_intrinsics.fy = msg->image_intrinsics.fy;
                    img_intrinsics.ppx = msg->image_intrinsics.ppx;
                    img_intrinsics.ppy = msg->image_intrinsics.ppy;
                    img_intrinsics.depth_units = msg->image_intrinsics.depth_units;

#if PUBLISH_METHOD
                    {
                        std::lock_guard<std::mutex> lock(write_mutex);
                        latest_times = TimesToAnalyze{
                            full_ms,
                            transport_ms,
                            0,
                            0
                        };

                        latest_frame = Frame{
                            img_color.clone(),
                            img_depth.clone(),
                            img_intrinsics,
                            msg->timestamp,
                            node->now().nanoseconds()
                        };
                    }
                    frame_cv.notify_one();
#else          
                    ImgAnalyze obj_msg;
                    obj_msg.has_object = 0;
#if GPU 
                    std::vector<Object> objects = process_image(detector, img_color, img_depth, img_intrinsics);

                    if (objects.size() > 0)
                    {
                        test += 1;
                        RCLCPP_INFO(node->get_logger(), "Frame %d.", test);
                        //RCLCPP_INFO(node->get_logger(), "%lu objects detected in the current frame.", objects.size());
                        obj_msg.has_object = 1;
                        obj_msg.object.resize(objects.size());
                        for (size_t i = 0; i < objects.size() && i < obj_msg.object.size(); ++i)
                        {
                            obj_msg.object[i].label = objects[i].label;
                            obj_msg.object[i].probability = objects[i].probability;
                            obj_msg.object[i].box.x = objects[i].rect.x;
                            obj_msg.object[i].box.y = objects[i].rect.y;
                            obj_msg.object[i].box.width = objects[i].rect.width;
                            obj_msg.object[i].box.height = objects[i].rect.height;
                            obj_msg.object[i].kps = objects[i].kps;
                            obj_msg.object[i].point.pose_x = objects[i].Pose2D.x;
                            obj_msg.object[i].point.pose_y = objects[i].Pose2D.y;
                            obj_msg.object[i].point3d.x = objects[i].Pose3D[0];
                            obj_msg.object[i].point3d.y = objects[i].Pose3D[1];
                            obj_msg.object[i].point3d.z = objects[i].Pose3D[2];
                        }
                    }                 
                        
#endif
                    obj_msg.header.stamp.sec = msg->timestamp / 1'000'000'000LL;
                    obj_msg.header.stamp.nanosec = msg->timestamp % 1'000'000'000LL;
                    obj_msg.middle_header.stamp = node->now();
                    RCLCPP_INFO(node->get_logger(), "Publishing image time: %f ms.", static_cast<double>(node->now().nanoseconds() - msg->timestamp)/1'000'000LL);

                    auto after = node->now();
                    auto process_duration = after.nanoseconds() - receive_ns;
                    process_time.push_back(static_cast<double>(process_duration) / 1'000'000'000.0); // Convert to seconds

                    pub->publish(obj_msg);
#endif
                    // ── Step 6: release chun  k back to iceoryx pool ────────────────────
                    // Must be called — otherwise the pool exhausts and publisher stalls.
                    iox_sub.release(userPayload);

                })
                .or_else([](auto &) {
                    // no chunk available — yield to avoid starving other threads
                    #if defined(__x86_64__) || defined(__i386__)
                        __asm__ volatile("pause" ::: "memory");
                    #elif defined(__aarch64__) || defined(__arm__)
                        __asm__ volatile("yield" ::: "memory");
                    #else
                        std::this_thread::yield();
                    #endif
                });
            }
        }
    }

#if SAVE_CSV
    save_csv(csv_path, times, object_identified);
#endif

    // ── shutdown summary ──────────────────────────────────────────────────────
    if (frame_count > 0) {
        const double avg_full      = total_full_ms      / static_cast<double>(frame_count);
        const double avg_transport = total_transport_ms / static_cast<double>(frame_count);
        RCLCPP_INFO(node->get_logger(), "──────────────────────────────────────────");
        RCLCPP_INFO(node->get_logger(), "Transport time summary");
        RCLCPP_INFO(node->get_logger(), "  Frames received      : %lu",   frame_count);
        RCLCPP_INFO(node->get_logger(), "  Full latency   A→D   : %.1f ms (%.3f us)" , avg_full,      avg_full * 1e3);
        RCLCPP_INFO(node->get_logger(), "  Transport only B→D   : %.1f ms (%.3f us)", avg_transport, avg_transport * 1e3);
        RCLCPP_INFO(node->get_logger(), "  Transport min B→D    : %.1f ms", min_transport_ms);
        RCLCPP_INFO(node->get_logger(), "  Transport max B→D    : %.1f ms", max_transport_ms);
        RCLCPP_INFO(node->get_logger(), "  CSV saved to         : %s",     csv_path.c_str());
        RCLCPP_INFO(node->get_logger(), " Average processing time: %ld ms", std::accumulate(times.begin(), times.end(), int64_t{0}, [](int64_t total, const TimesToAnalyze& tm){return total + tm.process_time_ms;}) / times.size());
        RCLCPP_INFO(node->get_logger(), " Average transfer time: %ld ms", std::accumulate(times.begin(), times.end(), int64_t{0}, [](int64_t total, const TimesToAnalyze& tm){return total + tm.transfer_thread_ms;}) / times.size());
        RCLCPP_INFO(node->get_logger(), "──────────────────────────────────────────");
    } else {
        RCLCPP_WARN(node->get_logger(), "No frames received");
    }
    
    RCLCPP_INFO(node->get_logger(), "Shutting down native iceoryx subscriber");
    stop = true;

#if PUBLISH_METHOD
    frame_cv.notify_all();
    process_image_and_pub.join();
#endif
    rclcpp::shutdown();
    return 0;
}
