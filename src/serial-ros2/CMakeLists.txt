cmake_minimum_required(VERSION 3.5)
project(serial)

find_package(ament_cmake REQUIRED)

# Build serial as SHARED and ensure -fPIC
add_library(${PROJECT_NAME} SHARED
    src/serial.cc
    src/impl/unix.cc
    src/impl/list_ports/list_ports_linux.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

# Force PIC
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Link platform-specific libraries
if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
    target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    target_link_libraries(${PROJECT_NAME} rt pthread)
elseif(WIN32)
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/win.cc
        src/impl/list_ports/list_ports_win.cc
    )
    target_link_libraries(${PROJECT_NAME} setupapi)
endif()

# Include headers
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Export for ament
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()

# Install
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(DIRECTORY include/serial DESTINATION include)
