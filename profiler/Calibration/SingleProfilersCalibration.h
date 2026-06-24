#pragma once
#include <iostream>
#include <sstream>
#include <limits>
#include <string>
#include <thread>
#include <chrono>
#include "profiler/calibration/ProfilerCalibrationInterfaces.h"
#include "profiler/parameters/RawImageParameters.h"
#include "profiler/api_util.h"

// Enumeration for target transformation axis
enum class TargetTransformAxis { X, Y, Unknown };

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

/* Discovers and connects  Mech-Eye 3D Laser Profilers for calibration.*/
std::optional<mmind::eye::Profiler> findAndConnectProfilerForCalibration()
{
    std::cout << "Find Mech-Eye 3D Laser Profilers..." << std::endl;
    std::vector<mmind::eye::ProfilerInfo> profilerInfoList =
        mmind::eye::Profiler::discoverProfilers();

    if (profilerInfoList.empty()) {
        std::cout << "No Mech-Eye 3D Laser Profilers found." << std::endl;
        return {};
    }

    for (size_t i = 0; i < profilerInfoList.size(); i++) {
        std::cout << "Mech-Eye 3D Laser Profiler index : " << i << std::endl;
        printProfilerInfo(profilerInfoList[i]);
    }

    std::string str;
    unsigned index = 0;
    std::cout << "Please enter the device index you want to choose as calibration profiler: "
              << std::endl;

    std::cin >> str;
    if (std::regex_match(str.begin(), str.end(), std::regex{"[0-9]+"}) &&
        static_cast<unsigned>(atoi(str.c_str())) < profilerInfoList.size()) {
        index = static_cast<unsigned>(atoi(str.c_str()));
    } else {
        std::cout << "Input invalid. Please enter the device index you want to connect: ";
        return {};
    }
    // Connect to selected profilers
    mmind::eye::Profiler profiler;
    auto status = profiler.connect(profilerInfoList[index]);
    if (!status.isOK()) {
        showError(status);
        return std::nullopt;
    }
    return profiler;
}
// Get device information and interact with the user for additional parameters
mmind::eye::DeviceInfo getDeviceInfo(mmind::eye::Profiler& profiler)
{
    mmind::eye::UserSet userSet = profiler.currentUserSet();
    mmind::eye::ProfilerInfo profilerInfo;
    auto status = profiler.getProfilerInfo(profilerInfo);
    if (!status.isOK()) {
        showError(status);
        return {};
    }
    printProfilerInfo(profilerInfo);

    // Get X-axis resolution
    double xResolution{};
    status = userSet.getFloatValue(mmind::eye::point_cloud_resolutions::XAxisResolution::name,
                                   xResolution);
    if (!status.isOK()) {
        showError(status);
        return {};
    }

    /*When scanning is triggered by an encoder, the Y - axis resolution can be calculated using the
    following equation :
    Y-axis resolution = encoder resolution * Trigger Interval / Trigger Signal Counting Mode * 4*/
    double yResolution{};
    status =
        userSet.getFloatValue(mmind::eye::point_cloud_resolutions::YResolution::name, yResolution);
    if (!status.isOK()) {
        showError(status);
        return {};
    }

    // Get ROI (Region of Interest) value
    mmind::eye::ProfileROI roiValue;
    status = userSet.getProfileRoiValue(mmind::eye::roi::ROI::name, roiValue);
    if (!status.isOK()) {
        showError(status);
        return {};
    }

    // Prompt user for downsampling intervals and camera motion direction
    std::cout << "\nEnter the downsampling interval in the X direction: " << std::endl;
    unsigned int downsampleX = getInputNumber<unsigned int>();

    std::cout << "\nEnter the downsampling interval in the Y direction: " << std::endl;
    unsigned int downsampleY = getInputNumber<unsigned int>();

    std::cout << "\nEnter the Camera motion direction: " << std::endl;
    bool directionPositive = getInputNumber<bool>();

    std::cout << "Please confirm the following device settings:" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;
    std::cout << "1. X-Axis Resolution (mm): " << float(xResolution / 1000) << std::endl;
    std::cout << "2. Y-Axis Resolution (mm): " << float(yResolution / 1000) << std::endl;
    std::cout << "3. Downsampling Factor (X): " << downsampleX << std::endl;
    std::cout << "4. Downsampling Factor (Y): " << downsampleY << std::endl;
    std::cout << "5. Motion Direction Sign: " << directionPositive << std::endl;
    std::cout << "6. ROI Size (Width , Height): (" << roiValue.width << " , " << roiValue.height
              << ")" << std::endl;
    std::cout << "7. ROI Center (X, Y): (" << roiValue.xAxisCenter << " , " << 0 << ")"
              << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    return {
        float(xResolution / 1000),
        float(yResolution / 1000),
        downsampleX,
        downsampleY,
        directionPositive,
        {static_cast<float>(roiValue.width), static_cast<float>(roiValue.height)},
        {static_cast<float>(roiValue.xAxisCenter), 0},
    };
}

// Print error messages when stitching fails
void printError(mmind::eye::MultiProfilerErrorStatus errorStatus)
{
    std::cout << "\nerrorStatus:" << errorStatus.errorCode << std::endl;
    std::cout << "\nerrorDescription:" << errorStatus.errorDescription << std::endl;
    std::cout << "\nerrorSource:" << errorStatus.errorSource << " " << errorStatus.groupID
              << std::endl;
}

// Set the calibration mode based on user input
mmind::eye::WidthExpansionType inputWidthExpansionCalibType()
{
    while (true) {
        std::cout << "\nEnter the number that represents the calibration types:"
                  << "\n1: S"
                  << "\n2: Z"
                  << "\n3: Disorder" << std::endl;

        switch (getInputNumber<int>()) {
        case 1:
            return mmind::eye::WidthExpansionType::S;
        case 2:
            return mmind::eye::WidthExpansionType::Z;
        case 3:
            return mmind::eye::WidthExpansionType::Disorder;
        default:
            std::cout << "Invalid input! Please enter 1��2 or 3." << std::endl;
        }
    }
}

// Set the calibration mode based on user input
mmind::eye::ProfilerCalibrationMode inputCalibType()
{
    while (true) {
        std::cout << "\nEnter the number that represents the calibration types:"
                  << "\n1: Wide"
                  << "\n2: Angle" << std::endl;

        switch (getInputNumber<int>()) {
        case 1:
            return mmind::eye::ProfilerCalibrationMode::Wide;
        case 2:
            return mmind::eye::ProfilerCalibrationMode::Angle;
        default:
            std::cout << "Invalid input! Please enter 1 or 2." << std::endl;
        }
    }
}

mmind::eye::ProfilerMovementAxis inputMovementAxis()
{
    while (true) {
        std::cout << "\nEnter the number that represents the movement axis:"
                  << "\n1: X"
                  << "\n2: Y"
                  << "\n3: Z" << std::endl;

        switch (getInputNumber<int>()) {
        case 1:
            return mmind::eye::ProfilerMovementAxis::X;
        case 2:
            return mmind::eye::ProfilerMovementAxis::Y;
        case 3:
            return mmind::eye::ProfilerMovementAxis::Z;
        default:
            std::cout << "Invalid input! Please enter 1,2 or 3 ." << std::endl;
        }
    }
}

// Set the target transformation axis based on user input
TargetTransformAxis inputTargetTransformAxis()
{
    while (true) {
        std::cout << "\nEnter the number that represents the transform axis:"
                  << "\n1: X"
                  << "\n2: Y" << std::endl;

        switch (getInputNumber<int>()) {
        case 1:
            return TargetTransformAxis::X;
        case 2:
            return TargetTransformAxis::Y;
        default:
            std::cout << "Invalid input! Please enter 1 or 2." << std::endl;
        }
    }
}

// Acquire profile data from a profiler
bool acquireProfileData(mmind::eye::Profiler& profiler, mmind::eye::ProfileBatch& totalBatch,
                        int captureLineCount, int dataWidth, bool isSoftwareTrigger)
{
    std::cout << "Start data acquisition." << std::endl;
    auto status = profiler.startAcquisition();
    if (!status.isOK()) {
        showError(status);
        return false;
    }

    if (isSoftwareTrigger) {
        status = profiler.triggerSoftware();
        if (!status.isOK()) {
            showError(status);
            return false;
        }
    }

    totalBatch.clear();
    totalBatch.reserve(captureLineCount);
    const int kMaxEmptyRetrievalCount = 15; // About 3 s with 200 ms sleep
    int emptyRetrievalCount = 0;
    while (totalBatch.height() < captureLineCount) {
        mmind::eye::ProfileBatch batch(dataWidth);
        status = profiler.retrieveBatchData(batch);
        if (status.isOK()) {
            if (batch.isEmpty()) {
                ++emptyRetrievalCount;
                if (emptyRetrievalCount >= kMaxEmptyRetrievalCount) {
                    std::cout << "No new data received for " << emptyRetrievalCount
                              << " consecutive retrievals. Stop waiting for data." << std::endl;
                    break;
                }
            } else {
                emptyRetrievalCount = 0;
                if (!totalBatch.append(batch))
                    break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } else if (!isSoftwareTrigger &&
                   status.errorCode ==
                       mmind::eye::ErrorStatus::MMIND_STATUS_TIMEOUT_ERROR) {
            // In external trigger mode, a retrieval timeout is expected when the trigger
            // signal stops. Treat it the same as an empty retrieval instead of failing
            // immediately. Other errors (e.g., device disconnect) should still cause
            // failure.
            ++emptyRetrievalCount;
            if (emptyRetrievalCount >= kMaxEmptyRetrievalCount) {
                std::cout << "No new data received for " << emptyRetrievalCount
                          << " consecutive retrievals. Stop waiting for data." << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } else {
            showError(status);
            return false;
        }
    }

    std::cout << "Stop data acquisition." << std::endl;
    status = profiler.stopAcquisition();
    if (!status.isOK())
        showError(status);
    return status.isOK();
}

// Asynchronously capture images from a profiler
mmind::eye::ProfilerImage captureAsync(mmind::eye::Profiler& profiler)
{
    mmind::eye::ProfilerInfo profilerInfo;
    showError(profiler.getProfilerInfo(profilerInfo));

    // Select "calib" user set to capture image.
    const std::string calibSetting{"calib"};
    mmind::eye::UserSetManager& userSetManager = profiler.userSetManager();
    std::string successMessage = "Set current set as the \"" + calibSetting + "\" user set.";
    showError(userSetManager.selectUserSet(calibSetting), successMessage);

    mmind::eye::UserSet userSet = profiler.currentUserSet();

    int dataWidth = 0;
    showError(
        userSet.getIntValue(mmind::eye::scan_settings::DataPointsPerProfile::name, dataWidth));
    int captureLineCount = 0;
    userSet.getIntValue(mmind::eye::scan_settings::ScanLineCount::name, captureLineCount);

    mmind::eye::ProfileBatch profileBatch(dataWidth);

    int dataAcquisitionTriggerSource{};
    showError(userSet.getEnumValue(mmind::eye::trigger_settings::DataAcquisitionTriggerSource::name,
                                   dataAcquisitionTriggerSource));

    //// Adjust the "Trigger Delay" appropriately to avoid interference between devices and ensure
    /// optimal imaging performance.
    // showError(userSet.setEnumValue(mmind::eye::trigger_settings::TriggerDelay::name, 100));

    bool isSoftwareTrigger =
        dataAcquisitionTriggerSource ==
        static_cast<int>(
            mmind::eye::trigger_settings::DataAcquisitionTriggerSource::Value::Software);

    if (!acquireProfileData(profiler, profileBatch, captureLineCount, dataWidth, isSoftwareTrigger))
        return {};

    if (profileBatch.checkFlag(mmind::eye::ProfileBatch::BatchFlag::Incomplete))
        std::cout << "Part of the batch's data is lost, the number of valid profiles is: "
                  << profileBatch.validHeight() << "." << std::endl;

    mmind::eye::ProfilerImage result;
    result.depth =
        cv::Mat(captureLineCount, dataWidth, CV_32FC1, profileBatch.getDepthMap().data()).clone();
    result.intensity =
        cv::Mat(captureLineCount, dataWidth, CV_8UC1, profileBatch.getIntensityImage().data())
            .clone();
    return result;
}

// Capture images from profiler
bool captureImages(mmind::eye::Profiler& profiler,
                   std::vector<mmind::eye::ProfilerImage>& profilerImages)
{
    while (true) {
        // Confirm capture with the user
        if (!confirmCapture()) {
            profiler.disconnect();
            break;
        }
        auto profilerImage = captureAsync(profiler);
        profilerImages.push_back(profilerImage);
    }
    return true;
}

// Capture images from profiler
bool captureImages(mmind::eye::Profiler& profiler,
                   std::vector<mmind::eye::ProfilerImage>& profilerImages,
                   mmind::eye::ProfilerImage& firstImage)
{
    bool isFirst = true;

    while (true) {
        if (!confirmCapture()) {
            profiler.disconnect();
            break;
        }

        auto img = captureAsync(profiler);

        if (isFirst) {
            firstImage = img;
            isFirst = false;
        } else {
            profilerImages.push_back(img);
        }
    }

    return true;
}
