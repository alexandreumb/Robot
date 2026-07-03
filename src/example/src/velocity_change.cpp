#include <atomic>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <time.h>
#include <mutex>    

#include "msgs/msg/teleop_command.hpp"

std::atomic<bool> running{true};
int velocity{0};
int angle{0};
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

int main(int argc, char ** argv)
{
    char c;

    rclcpp::init(argc, argv);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto node = rclcpp::Node::make_shared("teleop_publisher_node");

    auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
    auto publisher = node->create_publisher<msgs::msg::TeleopCommand>(
        "robot_steering_controller/reference_teleop", qos);

    std::thread pub_thread([&]() {
        while (running) {
            msgs::msg::TeleopCommand msg;
            msg.header.frame_id = "base_link";
            int a;
            int v;
            {
                std::lock_guard<std::mutex> lock(reader_mutex);
                v = velocity;
                a = angle;
            }   
            auto time = node->now().nanoseconds();
            msg.header.stamp.sec = time / 1'000'000'000LL;
            msg.header.stamp.nanosec = time % 1'000'000'000LL;
            msg.velocity = v;
            msg.angle = a;
            publisher->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    enableRawMode();

    while (running)
    {
        if (read(STDIN_FILENO, &c, 1) <= 0)
            continue;
        switch (c)
        {
            case '1':
                velocity = 1;
                break;
            case '2':
                velocity = 2;
                break;
            case '3':
                velocity = 3;
                break;
            case '4':
                velocity = 4;
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
                }
                break;
            case 'i':
                std::cout << "Current velocity: " << velocity << ", Current angle: " << angle << std::endl;
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
