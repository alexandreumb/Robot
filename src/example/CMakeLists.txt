#example
cmake_minimum_required(VERSION 3.8)
project(example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gnss_compass_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)
find_package(serial REQUIRED)
find_package(OpenCV REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_hoofs REQUIRED)
find_package(realsense2 REQUIRED)
find_package(msgs REQUIRED)
find_package(fixed_size_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

add_executable(gpio_verify src/gpio_verify.cpp)

# ----------------------------
# Camera nodes
# ----------------------------
add_executable(camera1_pub src/camera1_pub.cpp)
ament_target_dependencies(camera1_pub OpenCV rclcpp msgs realsense2)
target_link_libraries(camera1_pub iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh ${realsense2_LIBRARY})

add_executable(camera_bridge src/camera_bridge.cpp)
ament_target_dependencies(camera_bridge rclcpp msgs sensor_msgs OpenCV)
target_link_libraries(camera_bridge iceoryx_posh::iceoryx_posh iceoryx_hoofs::iceoryx_hoofs)

add_executable(velocity_change src/velocity_change.cpp)
ament_target_dependencies(velocity_change rclcpp msgs)

# Install Python modules
ament_python_install_package(robot_control)
install(PROGRAMS
  robot_control/control_validator.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(camera_sub_iceoryx_nativo src/camera_sub_iceoryx_nativo.cpp)
ament_target_dependencies(camera_sub_iceoryx_nativo rclcpp OpenCV msgs)
target_link_libraries(camera_sub_iceoryx_nativo iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh CUDA::cudart Yolo_Tensorrt)  # Link the YoloTensorrt library
target_include_directories(camera_sub_iceoryx_nativo PRIVATE
    /home/robotics4farmers/Dev/Robot/src/example
    /usr/local/cuda-12.6/targets/aarch64-linux/include
)
# add to CMakeLists.txt temporarily
# target_compile_options(camera1_sub_iceoryx_nativo PRIVATE -fsanitize=address)
# target_link_options(camera1_sub_iceoryx_nativo PRIVATE -fsanitize=address)
add_executable(camera_sub_iceoryx_no_thread src/camera_sub_iceoryx_no_thread.cpp)
ament_target_dependencies(camera_sub_iceoryx_no_thread rclcpp OpenCV msgs)
target_link_libraries(camera_sub_iceoryx_no_thread iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh CUDA::cudart Yolo_Tensorrt)  # Link the YoloTensorrt library
target_include_directories(camera_sub_iceoryx_no_thread PRIVATE
    /home/robotics4farmers/Dev/Robot/src/example
    /usr/local/cuda-12.6/targets/aarch64-linux/include
)

add_executable(camera_fastrtps_pub src/camera_fastrtps_pub.cpp)
ament_target_dependencies(camera_fastrtps_pub rclcpp OpenCV msgs fixed_size_msgs realsense2 sensor_msgs)
target_link_libraries(camera_fastrtps_pub iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh ${realsense2_LIBRARY})

add_executable(camera_fastrtps_sub src/camera_fastrtps_sub.cpp)
ament_target_dependencies(camera_fastrtps_sub rclcpp OpenCV msgs fixed_size_msgs sensor_msgs)
target_link_libraries(camera_fastrtps_sub iceoryx_hoofs::iceoryx_hoofs iceoryx_posh::iceoryx_posh)

# ----------------------------
# GPS hardware plugin (for simulation)
# ----------------------------
add_library(example SHARED 
    hardware/gps.cpp
    hardware/robot.cpp)
target_include_directories(example PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/example>
)
ament_target_dependencies(example
  rclcpp rclcpp_lifecycle hardware_interface controller_manager pluginlib serial gnss_compass_utils PkgConfig
)
pluginlib_export_plugin_description_file(hardware_interface example.xml)

ament_export_include_directories(include)
ament_export_libraries(example)
# ----------------------------
# Install targets
# ----------------------------
install(TARGETS
  camera1_pub camera_bridge camera_sub_iceoryx_nativo camera_sub_iceoryx_no_thread camera_fastrtps_pub camera_fastrtps_sub velocity_change
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY hardware/include/
  DESTINATION include/example
)

install(DIRECTORY description/urdf description/ros2_control_description description/launch
  DESTINATION share/example
)

install(DIRECTORY bringup/config
  DESTINATION share/example
)

install(TARGETS example
  EXPORT export_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_subdirectory(Yolo_Tensorrt)
add_subdirectory(Shared_Memory_Sensors)
# ----------------------------
# Testing / linting
# ----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
