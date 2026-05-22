cmake_minimum_required(VERSION 3.8)
project(gnss_compass_utils)

find_package(ament_cmake REQUIRED)

add_library(gnss_compass_utils STATIC
  src/an_packet_protocol.c
  src/ins_packets.c
)

target_include_directories(gnss_compass_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(gnss_compass_utils HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS gnss_compass_utils
  EXPORT gnss_compass_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()