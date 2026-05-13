#include <rclcpp/rclcpp.hpp>
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "fixed_size_msgs/msg/image8_mb.hpp"

#include <atomic>
#include <csignal>
#include <iostream>

using Image8Mb = fixed_size_msgs::msg::Image8Mb;

// Global stats
std::atomic<uint64_t> total_count{0};
std::atomic<double> total_sum_us{0.0};

// Signal flag
std::atomic<bool> stop_flag{false};

void signal_handler(int)
{
  stop_flag.store(true);
}

inline int64_t monotonic_now_ns()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

// ---------------- CAMERA NODE ----------------

class CameraNode : public rclcpp::Node
{
public:
  CameraNode(const rclcpp::NodeOptions & options)
  : Node("camera_node", options)
  {
    pub_ = this->create_publisher<Image8Mb>("image_raw", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&CameraNode::publish_image, this)
    );
  }

private:
  void publish_image()
  {
    auto msg = std::make_unique<Image8Mb>();

    msg->height = 480;
    msg->width = 640;
    msg->step = 640 * 3;
    
    std::fill(msg->data.begin(), msg->data.end(), 128);
    msg->timestamp = monotonic_now_ns();

    pub_->publish(std::move(msg));
  }

  rclcpp::Publisher<Image8Mb>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ---------------- PROCESSING NODE ----------------

class ProcessingNode : public rclcpp::Node
{
public:
  ProcessingNode(const rclcpp::NodeOptions & options)
  : Node("processing_node", options)
  {
    cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_;

    sub_ = this->create_subscription<Image8Mb>(
      "image_raw",
      10,
      std::bind(&ProcessingNode::callback, this, std::placeholders::_1),
      sub_options
    );
  }

private:
  void callback(const Image8Mb::SharedPtr msg)
  {
    auto received_ns = monotonic_now_ns();

    double full_us =
      static_cast<double>(received_ns - msg->timestamp) / 1000.0;

    // Atomic add using compare_exchange
    double current = total_sum_us.load(std::memory_order_relaxed);
    double desired;

    do {
      desired = current + full_us;
    } while (!total_sum_us.compare_exchange_weak(
      current, desired,
      std::memory_order_relaxed
    ));

    total_count.fetch_add(1, std::memory_order_relaxed);
  }

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<Image8Mb>::SharedPtr sub_;
};

// ---------------- MAIN ----------------

int main(int argc, char * argv[])
{
  std::signal(SIGINT, signal_handler);

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto camera_node = std::make_shared<CameraNode>(options);
  auto processing_node = std::make_shared<ProcessingNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);

  executor.add_node(camera_node);
  executor.add_node(processing_node);

  // Spin until Ctrl+C
  while (rclcpp::ok() && !stop_flag.load())
  {
    executor.spin_some();
  }

  executor.cancel();
  rclcpp::shutdown();

  // Compute average
  uint64_t count = total_count.load();
  double total_sum = total_sum_us.load();

  if (count > 0)
  {
    double avg = total_sum / static_cast<double>(count);
    std::cout << "\nAverage latency (us): " << avg << std::endl;
  }
  else
  {
    std::cout << "\nNo samples collected.\n";
  }

  return 0;
}