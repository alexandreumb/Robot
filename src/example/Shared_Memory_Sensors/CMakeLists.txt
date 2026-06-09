add_executable(SharedMemoryCompiler 
    shared_memory_compiler.cpp
)

target_link_libraries(SharedMemoryCompiler 
    nlohmann_json::nlohmann_json
)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/header_creator.h ${CMAKE_CURRENT_BINARY_DIR}/header_acessor.h
    COMMAND SharedMemoryCompiler ${CMAKE_CURRENT_SOURCE_DIR}/sharedmemory.json
    DEPENDS SharedMemoryCompiler ${CMAKE_CURRENT_SOURCE_DIR}/sharedmemory.json
    COMMENT "Currently compiling shared memory layout from sharedmemory.json"
)

add_custom_target(
    CompilerByproducts
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/header_creator.h ${CMAKE_CURRENT_BINARY_DIR}/header_acessor.h
)



add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/header_creator.h ${CMAKE_CURRENT_SOURCE_DIR}/header_acessor.h
    COMMAND ${CMAKE_COMMAND} -E copy 
            ${CMAKE_CURRENT_BINARY_DIR}/header_creator.h
            ${CMAKE_CURRENT_SOURCE_DIR}/header_creator.h
    COMMAND ${CMAKE_COMMAND} -E copy 
            ${CMAKE_CURRENT_BINARY_DIR}/header_accessor.h
            ${CMAKE_CURRENT_SOURCE_DIR}/header_accessor.h
    DEPENDS CompilerByproducts
    COMMENT "Currently copying compiled shared memory layout"
)

add_custom_target(
    Byproducts
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/header_creator.h ${CMAKE_CURRENT_SOURCE_DIR}/header_acessor.h
)


# Define the interface library SharedMemorySensors
add_library(SharedMemorySensors INTERFACE)
target_include_directories(SharedMemorySensors INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(SharedMemorySensors INTERFACE
    gnss_compass_utils
    Robotics_4_Farmers_CAN_IDs
    nlohmann_json::nlohmann_json
    ${realsense2_LIBRARY}
)

# Ensure that consumers of SharedMemorySensors depend on the generated headers
add_dependencies(SharedMemorySensors Byproducts)