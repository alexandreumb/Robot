# lib/Yolo_Tensorrt/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

add_library(Yolo_Tensorrt SHARED
    engine.cpp
    yolov8.cpp
)

# Include directories for this library
target_include_directories(Yolo_Tensorrt PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CUDA_INCLUDE_DIRS} 
    # ${TensorRT_INCLUDE_DIRS}
    )

# Link necessary libraries
target_link_libraries(Yolo_Tensorrt PUBLIC 
    ${OpenCV_LIBS} 
    SharedMemorySensors
    /usr/lib/x86_64-linux-gnu/libnvonnxparser.so
    )

# Define the export_to_engine executable
add_executable(export_to_engine 
    export_to_engine.cpp)

target_include_directories(export_to_engine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CUDA_INCLUDE_DIRS} 
    )

# Link the Yolo_Tensorrt library and necessary dependencies to the executable
target_link_libraries(export_to_engine PUBLIC
    ${OpenCV_LIBS} 
    Yolo_Tensorrt
    cudart 
    nvinfer 
    # nvinfer_plugin
    # rt
)







# add_executable(export_engine 
#     export_to_engine.cpp
# )

# add_library(Yolo_models INTERFACE) # Create an interface target for yolo library
# target_include_directories(Yolo_models INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
# target_link_libraries(Yolo_models INTERFACE ${CUDA_LIBRARIES} ${OpenCV_LIBS})

# set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/lib/Yolo_models" ${CMAKE_MODULE_PATH})

# add_library(tensorrt_cpp_api SHARED
#     engine.cpp
#     yolov8.cpp
#     )

# target_include_directories(tensorrt_cpp_api PUBLIC ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${TensorRT_INCLUDE_DIRS})
# target_link_libraries(tensorrt_cpp_api PUBLIC ${OpenCV_LIBS} ${CUDA_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${TensorRT_LIBRARIES})

# add_library(YoloV8_TRT SHARED yolov8.cpp)

# target_include_directories(YoloV8_TRT PUBLIC 
#     ${TensorRT_INCLUDE_DIRS} 
#     CMAKE_MODULE_PATH
#     )
# target_link_libraries(YoloV8_TRT PUBLIC 
#     tensorrt_cpp_api 
#     ${OpenCV_LIBS} 
#     ${TensorRT_LIBRARIES}
#     )

# target_include_directories(export_engine PUBLIC 
#     ${TensorRT_INCLUDE_DIRS} 
#     CMAKE_MODULE_PATH
# )
# target_link_libraries(export_engine PUBLIC 
#     tensorrt_cpp_api 
#     YoloV8_TRT
#     ${OpenCV_LIBS} 
#     ${TensorRT_LIBRARIES}
# )
