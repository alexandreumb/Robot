// Copyright 2020 ROS2-Control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// ...
#include <errno.h> 
#include <algorithm> 
#include <chrono> 
#include <csignal> 
#include <memory> 
#include <string> 
#include <thread> 
#include <vector> 
#include <atomic> 
#include <iostream> 
#include <sys/timerfd.h>
#include <unistd.h>
#include <sched.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
//#include <gpiod.h>
#include <fstream>

#include "controller_manager/controller_manager.hpp" 
#include "controller_manager/mmio_gpio.hpp"
#include "rclcpp/rclcpp.hpp" 
#include "realtime_tools/realtime_helpers.hpp" 
#include "controller_manager/memory.hpp"

using namespace std::chrono_literals; 
namespace { 
  int const kSchedPriority = 80; 
  std::atomic<bool> stop_loop(false); // Signal handler for Ctrl+C 
  
  void sigint_handler(int) 
  { 
    stop_loop = true; 
  } 
} // namespace 

struct CounterSample
{
    long long cycles;
    long long instructions;
    long long misses;
    double execution_us;
};

int main(int argc, char **argv) { 
  rclcpp::init(argc, argv); // Register Ctrl+C handler 
  std::signal(SIGINT, sigint_handler); 

  std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(); 
  std::string manager_node_name = "controller_manager"; 
  auto cm = std::make_shared<controller_manager::ControllerManager>(executor, manager_node_name); 
  const bool use_sim_time = cm->get_parameter_or("use_sim_time", false); 
  
  const int cpu_affinity = cm->get_parameter_or<int>("cpu_affinity", -1); 
  if (cpu_affinity >= 0) { 
    const auto affinity_result = realtime_tools::set_current_thread_affinity(cpu_affinity); 
    
    if (!affinity_result.first) { 
      RCLCPP_WARN( cm->get_logger(), "Unable to set the CPU affinity : '%s'", affinity_result.second.c_str()); 
    } 
  } 
  
  const bool has_realtime = realtime_tools::has_realtime_kernel(); 
  const bool lock_memory = cm->get_parameter_or<bool>("lock_memory", has_realtime); 
  
  if (lock_memory) { 
    const auto lock_result = realtime_tools::lock_memory(); 
    RCLCPP_INFO(cm->get_logger(), "lock memory message: %s", lock_result.second.c_str()); 

    if (!lock_result.first) { 
      RCLCPP_WARN(cm->get_logger(), "Unable to lock the memory: '%s'", lock_result.second.c_str()); 
    } 
  } 
  
  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate()); 
  
  const int thread_priority = cm->get_parameter_or<int>("thread_priority", kSchedPriority); 
  RCLCPP_INFO(cm->get_logger(), "Spawning %s RT thread with scheduler priority: %d", cm->get_name(), thread_priority); 
  const double target_ms = 10.0; // 50 Hz 

std::thread cm_thread([cm, thread_priority, use_sim_time, target_ms, cpu_affinity]() 
{      
    int iteration = 0;  // Add this back for tracking
    std::vector<CounterSample> samples;
    std::vector<double> diffs_;

    PerformanceCounters counters;
    //MmioGpio gpio;
    //gpio.configure_pactl();
    
    // Set CPU affinity FIRST (before RT scheduling)
    if (cpu_affinity >= 0) {
        const auto affinity_result = realtime_tools::set_current_thread_affinity(cpu_affinity);
        
        if (!affinity_result.first) {
            RCLCPP_WARN(cm->get_logger(), "Unable to set the CPU affinity: '%s'", affinity_result.second.c_str());
        } else {
            RCLCPP_INFO(cm->get_logger(), "CPU affinity set to core %d", cpu_affinity);
        }
    }
    
    // THEN set RT scheduling
    if (realtime_tools::has_realtime_kernel()) 
    { 
        if (!realtime_tools::configure_sched_fifo(thread_priority)) { 
            RCLCPP_WARN(cm->get_logger(), "Could not enable FIFO RT scheduling policy: with error number <%i>(%s)...", errno, strerror(errno)); 
        } 
        else 
        { 
            RCLCPP_INFO(cm->get_logger(), "Successful set up FIFO RT scheduling policy with priority %i.", thread_priority); 
        } 
    } 
    else 
    { 
        RCLCPP_WARN(cm->get_logger(), "No real-time kernel detected on this system..."); 
    }

    int policy;
    struct sched_param sp;

    pthread_getschedparam(
        pthread_self(),
        &policy,
        &sp
    );

    RCLCPP_INFO(
        cm->get_logger(),
        "policy=%d priority=%d",
        policy,
        sp.sched_priority
    );

    int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (tfd < 0) {
        RCLCPP_ERROR(cm->get_logger(), "timerfd_create failed");
        return;
    }

    auto const period_ns = 1'000'000'000LL / cm->get_update_rate();
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
        RCLCPP_ERROR(cm->get_logger(), "timerfd_settime failed: %s", strerror(errno));
        close(tfd);
        return;
    } 
    auto previous_time = std::chrono::steady_clock::now(); 

    while (rclcpp::ok() && !stop_loop)
    {
        ssize_t n = read(tfd, &expirations, sizeof(expirations));
        if (n < 0)
            break;

        auto current_time = std::chrono::steady_clock::now();

        auto measured_period =
            std::chrono::duration<double, std::milli>(
                current_time - previous_time).count();

        previous_time = current_time;

        iteration++;

        if (expirations > 1) {
            RCLCPP_WARN(
                cm->get_logger(),
                "Missed %llu deadline(s) at iteration %zu",
                expirations - 1,
                iteration);
        }

        // Duration calculated from steady_clock
        rclcpp::Duration dt(
            std::chrono::nanoseconds(
                static_cast<int64_t>(measured_period * 1e6)));

        auto current_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                current_time.time_since_epoch()
            ).count();

        // ROS time comes from the ROS clock
        rclcpp::Time ros_time(current_ns, RCL_STEADY_TIME);
        
        counters.start();
        cm->read(ros_time, dt);
        cm->update(ros_time, dt);
        cm->write(ros_time, dt);
        counters.stop();

        auto after = std::chrono::steady_clock::now();

        double execution_us =
            std::chrono::duration<double, std::milli>(
                after - current_time).count();

        samples.push_back({
            counters.read_counter(0),
            counters.read_counter(1),
            counters.read_counter(2),
            execution_us
        });

        diffs_.push_back(execution_us);
    }
    close(tfd);
     
    RCLCPP_WARN(
        cm->get_logger(),
        "Control loop average time %6f ms",
        ([&]() {
            double sum = 0.0;
            for (const auto& d : diffs_)
                sum += d;
            return (sum/iteration);
        })()
    );
    RCLCPP_INFO(cm->get_logger(), "Control loop completed %u iterations", iteration);

    /*
    std::ofstream file("diffs.csv");

    if (!file.is_open()) {
        RCLCPP_ERROR(cm->get_logger(), "Failed to open diffs.csv");
    } else {
        file << "sample,time_s\n";  // Header

        for (size_t i = 0; i < diffs_.size(); ++i) {
            file << i << "," << diffs_[i] << "\n";
        }
    }
    file.close();
    */

    std::ofstream file("perf_counters.csv");

    file << "iteration,cycles,instructions,cache_misses,execution_us\n";

    for (size_t i = 0; i < samples.size(); ++i)
    {
        file << i << ","
            << samples[i].cycles << ","
            << samples[i].instructions << ","
            << samples[i].misses << ","
            << samples[i].execution_us << "\n";
    }
    file.close();

});

  executor->add_node(cm); 
  executor->spin(); 
  cm_thread.join(); 
  rclcpp::shutdown(); 
  return 0; 
}
