#include <chrono>
#include <thread>
#include <future>
#include <unordered_set>
#include <opencv2/imgcodecs.hpp>
#include "profiler/api_util.h"
#include "profiler/parameters/RawImageParameters.h"
#include "MultipleProfilersCalibrationUtil.h"

namespace profiler_calibration_util {

// Get user input as a string
std::string getInputCommand()
{
    std::string command;
    std::cin.clear();
    std::cin >> command;
    if (std::cin.rdbuf()->in_avail() != 0) {
        std::cin.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
    }
    return command;
}

/*Prompts the user to input a boolean valueand validates the input.
Accepts "true", "false", "1", or "0" (case-insensitive).*/
bool getInputBool()
{
    std::string inputStr;
    while (true) {
        std::cin.clear();
        std::getline(std::cin, inputStr);
        std::transform(inputStr.begin(), inputStr.end(), inputStr.begin(), ::tolower);
        if (inputStr == "true" || inputStr == "1") {
            return true;
        } else if (inputStr == "false" || inputStr == "0") {
            return false;
        } else {
            std::cout << "Invalid input. Please enter 'true', 'false', '1', or '0'." << std::endl;
        }
    }
}

// Prompt the user to select a command type
CommandType enterCommand()
{
    std::cout << "\nEnter the letter that represents the action you want to perform." << std::endl;
    std::cout << "C: Calibration and stitching using real-time camera capture." << std::endl;
    std::cout << "S: Stitching using historical calibration files with single major device."
              << std::endl;
    std::cout << "M: Stitching using historical calibration files with multi major device."
              << std::endl;
    const auto command = getInputCommand();

    if (command == "C" || command == "c") {
        return CommandType::Calibration;
    }
    if (command == "S" || command == "s") {
        return CommandType::StitchSingleMajorDevice;
    }

    if (command == "M" || command == "m") {
        return CommandType::StitchMultiMajorDevice;
    }
    return CommandType::Unknown;
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

// Set the cloud stitching option
int inputCloudStitchOption()
{
    while (true) {
        std::cout << "\nEnter the number that represents the cloud stitching option:"
                  << "\n1: Refine stitching parameters and do overlap region uniform sampling"
                  << "\n2: Only do overlap region uniform sampling"
                  << "\n3: Do neither of the above processes" << std::endl;
        switch (getInputNumber<int>()) {
        case 1:
            return static_cast<int>(mmind::eye::ProfilerCalibrationInterfaces::CloudStitchOption::
                                        RefineStitchParameters) |
                   static_cast<int>(mmind::eye::ProfilerCalibrationInterfaces::CloudStitchOption::
                                        OverlapRegionUniformSampling);
        case 2:
            return static_cast<int>(mmind::eye::ProfilerCalibrationInterfaces::CloudStitchOption::
                                        OverlapRegionUniformSampling);
        case 3:
            return 0;
        default:
            break;
        }
    }
}

// Print error messages when stitching fails
void printError(mmind::eye::MultiProfilerErrorStatus errorStatus)
{
    std::cout << "\nerrorStatus:" << errorStatus.errorCode << std::endl;
    std::cout << "\nerrorDescription:" << errorStatus.errorDescription << std::endl;
    std::cout << "\nerrorSource:" << errorStatus.errorSource << " " << errorStatus.groupID
              << std::endl;
}

/* Discovers and connects multiple Mech-Eye 3D Laser Profilers for calibration.
 * Ensures that all connected profilers are of the same model for calibration.*/
std::vector<mmind::eye::Profiler> findAndConnectMultiProfilerForCalibration()
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
    std::vector<unsigned> indices;
    std::unordered_set<unsigned> uniqueIndices; // Prevent duplicate inputs
    while (true) {
        if (indices.empty()) {
            std::cout << "Please enter the device index you want to choose as major profiler: "
                      << std::endl;
        } else {
            std::cout << "Please enter the device index you want to choose as minor profiler: "
                      << std::endl;
            std::cout << "Enter the character 'c' to terminate adding devices" << std::endl;
        }

        std::cin >> str;
        if (str == "c" && indices.size() > 1)
            break;
        if (std::regex_match(str.begin(), str.end(), std::regex{"[0-9]+"}) &&
            static_cast<unsigned>(atoi(str.c_str())) < profilerInfoList.size()) {
            unsigned index = static_cast<unsigned>(atoi(str.c_str()));

            // Ensure all devices are of the same model
            if (!indices.empty() &&
                profilerInfoList[index].model != profilerInfoList[indices[0]].model) {
                std::cout << "Input invalid. Please choose the same model device to connect: ";
                continue;
            }
            if (uniqueIndices.insert(index).second) { // Insert successful, no duplicate
                indices.push_back(index);
            }
        } else {
            std::cout << "Input invalid. Please enter the device index you want to connect: ";
        }
    }

    // Connect to selected profilers
    std::vector<mmind::eye::Profiler> profilerList{};
    for (auto index : indices) {
        mmind::eye::Profiler profiler;
        auto status = profiler.connect(profilerInfoList[index]);
        if (status.isOK())
            profilerList.push_back(profiler);
        else
            showError(status);
    }
    return profilerList;
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
    while (totalBatch.height() < captureLineCount) {
        mmind::eye::ProfileBatch batch(dataWidth);
        status = profiler.retrieveBatchData(batch);
        if (status.isOK()) {
            if (!totalBatch.append(batch))
                break;
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
    std::cout << "1. X-Axis Resolution (um): " << float(xResolution / 1000) << std::endl;
    std::cout << "2. Y-Axis Resolution (um): " << float(yResolution / 1000) << std::endl;
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

// Convert a 3x4 matrix to a 4x4 homogeneous matrix
cv::Matx44f convert34To44(const cv::Matx34f& mat34)
{
    cv::Matx44f mat44;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            mat44(i, j) = mat34(i, j);
        }
    }
    mat44(3, 0) = 0.0f;
    mat44(3, 1) = 0.0f;
    mat44(3, 2) = 0.0f;
    mat44(3, 3) = 1.0f;
    return mat44;
}

// Truncate a 4x4 homogeneous matrix to a 3x4 matrix (remove the last row)
cv::Matx34f convert44To34(const cv::Matx44f& mat44)
{
    cv::Matx34f mat34;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            mat34(i, j) = mat44(i, j);
        }
    }
    return mat34;
}

// Capture images from each profiler in the list
bool captureImagesForEachProfiler(std::vector<mmind::eye::Profiler>& profilerList,
                                  mmind::eye::ProfilerImage& majorImage,
                                  std::vector<cv::Mat>& minorDepths,
                                  std::vector<mmind::eye::ProfilerImage>& minorImages)
{
    // Validate the profiler list
    if (profilerList.empty()) {
        std::cout << "No profilers connected." << std::endl;
        return false;
    }

    // Confirm capture with the user
    if (!confirmCapture()) {
        for (auto& profiler : profilerList)
            profiler.disconnect();
        return false;
    }

    std::vector<std::future<mmind::eye::ProfilerImage>> container;

    // Start asynchronous capture for each profiler
    for (size_t i = 0; i < profilerList.size(); ++i) {
        container.emplace_back(
            std::async(std::launch::async, captureAsync, std::ref(profilerList[i])));
    }

    // Process the results
    try {
        for (size_t i = 0; i < container.size(); ++i) {
            mmind::eye::ProfilerImage result = container[i].get();
            if (i == 0) {
                majorImage.depth = result.depth.clone(); // Use clone to ensure data independence
                majorImage.intensity = result.intensity.clone();
            } else {
                minorDepths.emplace_back(result.depth.clone());
                minorImages.emplace_back(
                    mmind::eye::ProfilerImage{result.depth.clone(), result.intensity.clone()});
            }
            profilerList[i].disconnect();
        }
    } catch (const std::exception& e) {
        std::cerr << "Capture failed: " << e.what() << std::endl;
        return false;
    }

    return true;
}

bool loadCalibration(mmind::eye::ProfilerCalibrationInterfaces& calibInstance,
                     const std::string& prompt)
{
    std::string filePath;

    // Display the user prompt and get file path input
    std::cout << prompt;
    std::cin >> filePath;

    // Attempt to load calibration properties from the specified file
    auto loadErrorStatus = calibInstance.loadCalibProperties(filePath);

    // Handle loading failure
    if (!loadErrorStatus.isOK()) {
        std::cerr << "Failed to load calibration from " << filePath << std::endl;
        return false;
    }
    return true;
}

cv::Matx34f computeTransformProduct(const cv::Matx34f& baseTransform,
                                    const cv::Matx34f& additionalTransform)
{
    // Convert 3x4 matrices to 4x4 homogeneous coordinates
    cv::Matx44f baseTransform44 = convert34To44(baseTransform);
    cv::Matx44f additionalTransform44 = convert34To44(additionalTransform);

    // Matrix multiplication: Result = baseTransform * additionalTransform
    // Note: OpenCV uses left-to-right multiplication order (column-major)
    cv::Matx44f result44 = baseTransform44 * additionalTransform44;

    // Convert back to 3x4 matrix format
    return convert44To34(result44);
}
} // namespace profiler_calibration_util