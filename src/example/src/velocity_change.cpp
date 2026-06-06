#include <atomic>
#include <csignal>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "msgs/msg/img_analyze_msg.hpp"

std::atomic<bool> running{true};
std::atomic<int> velocity{0};

void signal_handler(int) { running = false; }


void publish_velocity()
{
    auto qos = rclcpp::QoS(1)
        .best_effort()
        .durability_volatile();

    auto node = rclcpp::Node::make_shared("velocity_publisher_node");
    auto publisher = node->create_publisher<msgs::msg::ImgAnalyzeMsg>("robot_steering_controller/reference", qos);

    msgs::msg::ImgAnalyzeMsg msg;
    msg.object.resize(1);  // Preallocate one object for simplicity

    msg.object[0].name = "person";
    msg.object[0].distance = 0;
    msg.object[0].confidence = 0.9;
    msg.header.stamp.sec = 0;
    msg.header.stamp.nanosec = 1;
    msg.header.frame_id = "camera_frame";
    
    while (running)
    {
        msg.velocity = velocity.load();
        publisher->publish(std::move(msg));
    
        printf("Publishing velocity: %d\n", velocity.load());

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1000));
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::thread pub_thread(publish_velocity);

    while (running)
    {
        int c = getchar();

        switch (c)
        {
        case '0': velocity = 0; break;
        case '1': velocity = 1; break;
        case '2': velocity = 2; break;
        case '3': velocity = 3; break;
        case '4': velocity = 4; break;
        case '5': velocity = 5; break;
        case '6': velocity = 6; break;
        case 'x': running = false; break;
        }
    }

    pub_thread.join();

    rclcpp::shutdown();
    return 0;
}