#include <atomic>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <time.h>
#include <mutex>    
#include <sys/timerfd.h>

#include "msgs/msg/teleop_command.hpp"

#define USE_CPU_AFFINITY     1
#define USE_RT_SCHEDULING    0

constexpr int      SUBSCRIBER_CORE = 3;
constexpr int      RT_PRIORITY     = 40;

std::atomic<bool> running{true};
double velocity{0};
int angle{0};
int manual_override{0}; // 0 = no manual override, 1 = manual override
int backwards{1}; // 1 = forward, -1 = backward
struct termios oldt, newt;
std::mutex reader_mutex;    

void enableRawMode()
{
    // Get current terminal settings
    tcgetattr(STDIN_FILENO, &oldt); 
    newt = oldt;

    // Disable canonical mode and echo
    newt.c_lflag &= ~(ICANON | ECHO);

    // Apply new settings
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

void disableRawMode()
{
    // Restore old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

//Signal Handler
void signal_handler(int) 
{ 
    disableRawMode();
    running = false; 
}

/*
inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}
*/
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
    char c;

    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    configure_thread();
    auto node = rclcpp::Node::make_shared("teleop_publisher_node");

    auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
    auto publisher = node->create_publisher<msgs::msg::TeleopCommand>(
        "robot_steering_controller/reference_teleop", qos);

    std::thread pub_thread([&]() {
        int times{0};
        int executions{0};

        int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
        if (tfd < 0) {
            return 0;
        }

        auto const period_ns = 1'000'000'000LL / 100;
        timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        uint64_t first_abs_ns =
        (uint64_t)now.tv_sec * 1000000000ULL +
        now.tv_nsec +
        period_ns;

        itimerspec ts{};
        ts.it_value.tv_sec  = first_abs_ns / 1'000'000'000LL;
        ts.it_value.tv_nsec = first_abs_ns % 1'000'000'000LL;
        ts.it_interval.tv_sec  = period_ns / 1'000'000'000LL;
        ts.it_interval.tv_nsec = period_ns % 1'000'000'000LL;
        
        uint64_t expirations;

        if (timerfd_settime(tfd, TFD_TIMER_ABSTIME, &ts, nullptr) < 0) 
        {
            close(tfd);
            return 0;
        } 

        while (running) {

            ssize_t n = read(tfd, &expirations, sizeof(expirations));
            if (n < 0)
                break;
            msgs::msg::TeleopCommand msg;
            msg.header.frame_id = "base_link";
            int a;
            int v;
            int manual;
            {
                std::lock_guard<std::mutex> lock(reader_mutex);
                v = velocity;
                a = angle;
                manual = manual_override;
            }   
            auto time = node->now().nanoseconds();
            msg.header.stamp.sec = time / 1'000'000'000LL;
            msg.header.stamp.nanosec = time % 1'000'000'000LL;
            msg.velocity = v;
            msg.angle = a;
            msg.manual = manual;
            publisher->publish(msg);
        }

    });

    enableRawMode();

    while (running)
    {
        if (read(STDIN_FILENO, &c, 1) <= 0)
            continue;
        switch (c)
        {
            case '0':
                velocity = 0;
                break;
            case '1':
                velocity = 1.0 * backwards;
                break;
            case '2':
                velocity = 2.0 * backwards;
                break;
            case '3':
                velocity = 3.0 * backwards;
                break;
            case '4':
                velocity = 4.0 * backwards;
                break;
            case 27:
                if (read(STDIN_FILENO, &c, 1) <= 0)
                    continue;
                if (c == '[')
                {
                    if (read(STDIN_FILENO, &c, 1) <= 0)
                        continue;                   
                    if (c == 'C')
                        angle = 15;
                    else if (c == 'D')
                        angle = -15;
                    else if (c == 'A')
                        angle = 0;
                    else if (c == 'B') {

                        backwards = -backwards; // Toggle forward/backward
                        velocity = velocity * -1.0; // Reset velocity when changing direction
                        std::cout << "Backwards: " << (backwards == -1 ? "YES" : "NO") << std::endl;
                    }
                }
                break;
            case 'm':
                manual_override = 1 - manual_override; // Toggle manual override
                break;
            case 'i':
                std::cout << "Current velocity: " << velocity << ", Current angle: " << angle << ", Manual override: " << (manual_override ? "ON" : "OFF") << ", Backwards: " << (backwards == -1 ? "YES" : "NO") << std::endl;
                break;
            case 'q':
                running = false;
                break;
            default:
                break;;
        }
    }

    disableRawMode();
    pub_thread.join();
    rclcpp::shutdown();
    return 0;
}
