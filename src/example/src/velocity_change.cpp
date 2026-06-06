#include <atomic>
#include <csignal>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "msgs/msg/img_analyze_msg.hpp"

std::atomic<bool> running{true};
std::atomic<int> velocity{0};

void signal_handler(int) { running = false; }

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto node = rclcpp::Node::make_shared("velocity_publisher_node");

    auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
    auto publisher = node->create_publisher<msgs::msg::ImgAnalyzeMsg>(
        "robot_steering_controller/reference", qos);

    msgs::msg::ImgAnalyzeMsg msg;
    msg.object.resize(1);
    msg.object[0].name = "person";
    msg.object[0].distance = 0;
    msg.object[0].confidence = 0.9;
    msg.header.frame_id = "camera_frame";

    std::thread pub_thread([&]() {
        while (running) {
            msg.header.stamp = node->now();
            msg.velocity = velocity.load();
            publisher->publish(msg);
            printf("Publishing velocity: %d\n", velocity.load());
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
            case '6': velocity = 6; break;
            case 'x': running = false; break;
            default: break;
        }
    }

    pub_thread.join();
    rclcpp::shutdown();
    return 0;
}