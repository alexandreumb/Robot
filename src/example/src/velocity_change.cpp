#include <atomic>
#include <csignal>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <time.h>

#include "msgs/msg/velocity.hpp"

std::atomic<bool> running{true};
std::atomic<int> velocity{0};

void signal_handler(int) { running = false; }

inline int64_t monotonic_now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto node = rclcpp::Node::make_shared("velocity_publisher_node");

    auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
    auto publisher = node->create_publisher<msgs::msg::Velocity>(
        "robot_steering_controller/reference_velocity", qos);

    msgs::msg::Velocity msg;
    msg.header.frame_id = "base_link";

    std::thread pub_thread([&]() {
        while (running) {
            auto time = monotonic_now_ns();
            msg.header.stamp.sec = time / 1'000'000'000LL;
            msg.header.stamp.nanosec = time % 1'000'000'000LL;
            msg.velocity = velocity.load();
            printf("Publishing velocity: %d\n", velocity.load());
            publisher->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    printf("Controls: 0-6 = velocity, x = quit\n");
    while (running) {
        int c = getchar();
        switch (c) {
            case '0': velocity = 0; break;
            case '1': velocity = 1; break;
            case '2': velocity = 2; break;
            case '3': velocity = 3; break;
            case '4': velocity = 4; break;
            case '5': velocity = 5; break;
            case '6': velocity = -1; break;
            case 'x': running = false; break;
            default: break;
        }
    }

    pub_thread.join();
    rclcpp::shutdown();
    return 0;
}