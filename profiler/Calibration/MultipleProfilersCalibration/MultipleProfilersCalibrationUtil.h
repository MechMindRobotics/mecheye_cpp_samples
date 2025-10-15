#pragma once
#include <iostream>
#include "profiler/calibration/ProfilerCalibrationInterfaces.h"
namespace profiler_calibration_util {

// Enumeration for command types
enum class CommandType {
    Calibration,             ///< Calibration command
    StitchSingleMajorDevice, ///< Single major device stitching mode
    StitchMultiMajorDevice,  ///< Multiple major devices stitching mode
    Unknown,                 ///< Unknown command type
};

// Enumeration for target transformation axis
enum class TargetTransformAxis { X, Y, Unknown };

// Get keyboard input as a string
std::string getInputCommand();

// Get keyboard input as a bool value
bool getInputBool();

// Get keyboard input as a number
template <typename T>
T getInputNumber()
{
    if (std::cin.rdbuf()->in_avail() != 0) {
        std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
    }
    T input;
    std::string inputStr;
    while (true) {
        std::cin.clear();
        std::getline(std::cin, inputStr);
        std::istringstream iss(inputStr);
        if (iss >> input && iss.eof()) {
            break;
        } else {
            std::cout << "Invalid input. Please enter a valid number." << std::endl;
        }
    }
    return input;
}

// Select operation type based on user input
CommandType enterCommand();

// Set calibration mode (camera installation method)
mmind::eye::ProfilerCalibrationMode inputCalibType();

// Set the target transformation axis
TargetTransformAxis inputTargetTransformAxis();

// Set the cloud stitching option
int inputCloudStitchOption();

// Print error message when stitching fails
void printError(mmind::eye::MultiProfilerErrorStatus errorStatus);

// Find and connect multiple Mech-Eye 3D Laser Profiler devices
std::vector<mmind::eye::Profiler> findAndConnectMultiProfilerForCalibration();

// Acquire profile data
bool acquireProfileData(mmind::eye::Profiler& profiler, mmind::eye::ProfileBatch& totalBatch,
                        int captureLineCount, int dataWidth, bool isSoftwareTrigger);

//  Get device information and interact with user to input downsampling and camera movement
//  direction parameters
mmind::eye::DeviceInfo getDeviceInfo(mmind::eye::Profiler& profiler);

// Asynchronously capture images (thread-safe)
mmind::eye::ProfilerImage captureAsync(mmind::eye::Profiler& profiler);

// Convert a 3x4 matrix to a 4x4 homogeneous matrix
cv::Matx44f convert34To44(const cv::Matx34f& mat34);

// Truncate a 4x4 homogeneous matrix to a 3x4 matrix (remove the last row)
cv::Matx34f convert44To34(const cv::Matx44f& mat44);

// Capture images for each profiler
bool captureImagesForEachProfiler(std::vector<mmind::eye::Profiler>& profilerList,
                                  mmind::eye::ProfilerImage& majorImage,
                                  std::vector<cv::Mat>& minorDepths,
                                  std::vector<mmind::eye::ProfilerImage>& minorImages);

// Load calibration properties from a file into the calibration interface instance.
bool loadCalibration(mmind::eye::ProfilerCalibrationInterfaces& calibInstance,
                     const std::string& prompt);

/**
 * Computes the combined transformation matrix by multiplying two input transformation matrices.
 * The multiplication follows OpenCV's matrix multiplication order (baseTransform *
 * additionalTransform).
 *
 * @param baseTransform        The first transformation matrix (typically representing the base
 * frame).
 * @param additionalTransform The second transformation matrix (to be applied relative to the base).
 * @return                     The combined transformation matrix as a 3x4 matrix.
 */
cv::Matx34f computeTransformProduct(const cv::Matx34f& baseTransform,
                                    const cv::Matx34f& additionalTransform);
} // namespace profiler_calibration_util