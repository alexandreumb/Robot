// #include <iostream>
#include <opencv2/opencv.hpp>
#include "yolov8.h"

#if defined(__aarch64__) || defined(_M_ARM64)
const std::string onnxModelPath = "/home/robotics4farmers/R4F_EVAbot_Navigation/lib/Yolo_Tensorrt/r4f_yolov8m_seg.onnx";
#elif defined(__x86_64__) || defined(_M_X64)
const std::string onnxModelPath = "/home/guilh/R4F_EVAbot_Navigation/lib/Yolo_Tensorrt/r4f_pc_yolov8m_seg.onnx";
#endif

int main()
{
    std::cout << "yolo being created" << std::endl;
    YoloV8Config config;
    YoloV8 yoloV8(onnxModelPath, config);
    std::cout << "yolo created" << std::endl;
    return 0;
}