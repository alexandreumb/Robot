// config.h
#ifndef CONFIG_H
#define CONFIG_H
#include <string>

constexpr bool isUnix = true; // Change this to false for non-Unix

enum class CameraMode {
    Real,
    Sim1
};

enum class GPSMode {
    Real,
    Sim1,
    Sim2
};

enum class CANMode {
    Real,
    Sim1,
    Sim2
};

constexpr CameraMode cameraMode = CameraMode::Sim1;
constexpr GPSMode gpsMode = GPSMode::Sim1;
constexpr CANMode canMode = CANMode::Sim1;
const std::string camera_bag_dir = "/home/guilh/Data_Vineyard/Vinha-09-04/row1.bag";

#endif // CONFIG_H