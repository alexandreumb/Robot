// file created by cyrusbehr at 04/2024
// adapted by G-S-Rodrigues 06/2024

#pragma once
#include "engine.h"
#include "header_accessor.h"
#include <fstream>

// Utility method for checking if a file exists on disk
inline bool doesFileExist(const std::string &name) {
    std::ifstream f(name.c_str());
    return f.good();
}

struct Object {
    // The object class.
    int label{};
    // The detection's confidence probability.
    float probability{};
    // The object bounding box rectangle.
    cv::Rect_<float> rect;
    // Semantic segmentation mask
    cv::Mat boxMask;
    // Pose estimation key points
    std::vector<float> kps{};
    // 2D Pose estimation 
    cv::Point Pose2D{};
    // 3D Pose estimation 
    std::vector<float> Pose3D{std::vector<float>(3)};
};


// Config the behavior of the YoloV8 detector.
// Can pass these arguments as command line parameters.
struct YoloV8Config {
    // The precision to be used for inference
    Precision precision = Precision::FP16; //FP16
    // Calibration data directory. Must be specified when using INT8 precision.
    std::string calibrationDataDirectory;
    // Probability threshold used to filter detected objects
    float probabilityThreshold = 0.70f; // 0.25f
    // Non-maximum suppression threshold
    float nmsThreshold = 0.80f; // 0.65f
    // Max number of detected objects to return
    int topK = 50; //100
    // Segmentation config options
    int segChannels = 32;
    int segH = 160;
    int segW = 160;
    float segmentationThreshold = 0.5f;
    // Pose estimation options
    int numKPS = 17;
    float kpsThreshold = 0.5f;
    // Class thresholds (default are COCO classes)
    std::vector<std::string> classNames = {
        "Person", "Post", "Trunk"};
};

class YoloV8 {
public:
    // Builds the onnx model into a TensorRT engine, and loads the engine into memory
    YoloV8(const std::string &onnxModelPath, const YoloV8Config &config);

    // Detect the objects in the image
    std::vector<Object> detectObjects(const cv::Mat &inputImageBGR);
    std::vector<Object> detectObjects(const cv::cuda::GpuMat &inputImageBGR);
    std::vector<Object> extractObjects(const cv::Mat &depth_img, std::vector<Object> objects, image_intrinsics image_intrinsics, float maximum_detection_threshold);

    // Draw the object bounding boxes and labels on the image
    void drawObjectLabels(cv::Mat &image, const std::vector<Object> &objects, unsigned int scale = 2);

private:
    // Preprocess the input
    std::vector<std::vector<cv::cuda::GpuMat>> preprocess(const cv::cuda::GpuMat &gpuImg);

    // Postprocess the output
    std::vector<Object> postprocessDetect(std::vector<float> &featureVector);

    // Postprocess the output for pose model
    std::vector<Object> postprocessPose(std::vector<float> &featureVector);

    // Postprocess the output for segmentation model
    std::vector<Object> postProcessSegmentation(std::vector<std::vector<float>> &featureVectors);

    std::unique_ptr<Engine<float>> m_trtEngine = nullptr;

    // Used for image preprocessing
    // YoloV8 model expects values between [0.f, 1.f] so we use the following params
    const std::array<float, 3> SUB_VALS{0.f, 0.f, 0.f};
    const std::array<float, 3> DIV_VALS{1.f, 1.f, 1.f};
    const bool NORMALIZE = true;

    float m_ratio = 1;
    float m_imgWidth = 0;
    float m_imgHeight = 0;

    // Filter thresholds
    const float PROBABILITY_THRESHOLD;
    const float NMS_THRESHOLD;
    const int TOP_K;

    // Segmentation constants
    const int SEG_CHANNELS;
    const int SEG_H;
    const int SEG_W;
    const float SEGMENTATION_THRESHOLD;

    // Object classes as strings
    const std::vector<std::string> CLASS_NAMES;

    // Pose estimation constant
    const int NUM_KPS;
    const float KPS_THRESHOLD;

    // Color list for drawing objects
    const std::vector<std::vector<float>> COLOR_LIST = {{1, 0, 0},
                                                        {0, 1, 0},
                                                        {0, 0, 1}};

    const std::vector<std::vector<unsigned int>> KPS_COLORS = {{1, 0, 0},
                                                        {0, 1, 0},
                                                        {0, 0, 1}};

    const std::vector<std::vector<unsigned int>> SKELETON = {{16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13}, {6, 12}, {7, 13},
                                                             {6, 7},   {6, 8},   {7, 9},   {8, 10},  {9, 11},  {2, 3},  {1, 2},
                                                             {1, 3},   {2, 4},   {3, 5},   {4, 6},   {5, 7}};

    const std::vector<std::vector<unsigned int>> LIMB_COLORS = {{1, 0, 0},
                                                        {0, 1, 0},
                                                        {0, 0, 1}};
};