#include <thread>
#include <future>
#include <chrono>
#include <opencv2/highgui/highgui.hpp>
#include "profiler/api_util.h"
#include "MultipleProfilersCalibrationUtil.h"

using namespace mmind::eye;
using Calibration = mmind::eye::ProfilerCalibrationInterfaces;
namespace Util = profiler_calibration_util;
namespace {
bool captureImages(std::vector<mmind::eye::Profiler>& profilerList, ProfilerImage& majorImage,
                   std::vector<cv::Mat>& minorDepths, std::vector<ProfilerImage>& minorImages)
{
    if (!Util::captureImagesForEachProfiler(profilerList, majorImage, minorDepths, minorImages)) {
        std::cout << "Capture failed, please make sure each device can capture normally."
                  << std::endl;
        return false;
    }
    return true;
}

void stitchAndFuseImages(Calibration& calibInstance, const ProfilerImage& majorImage,
                         const std::vector<ProfilerImage>& minorImages,
                         const std::vector<CalibResult>& calibResults, const std::string& filePath,
                         bool& ifExit)
{
    // Variables to store stitching status and results
    MultiStitchResultZParallel stitchResults;

    // Perform image stitching for Z-parallel calibration
    auto stitchStatus = calibInstance.stitchImagesForZParallel(majorImage, minorImages,
                                                               calibResults, stitchResults);

    // Check if stitching was successful
    if (!stitchStatus.isOK()) {
        Util::printError(stitchStatus);
        ifExit = true;
        return;
    }

    // Perform cloud stitching
    const auto cloudStitchOption = Util::inputCloudStitchOption();
    stitchStatus = calibInstance.stitchPointCloud(
        majorImage, minorImages, cloudStitchOption,
        static_cast<int>(Calibration::StitchParamRefineOption::RefineT), calibResults);

    // Check if stitching was successful
    if (!stitchStatus.isOK()) {
        Util::printError(stitchStatus);
        ifExit = true;
        return;
    }

    // Prompt whether to save calibration files if stitching parameters were refined
    if ((cloudStitchOption &
         static_cast<int>(Calibration::CloudStitchOption::RefineStitchParameters)) != 0) {
        std::cout << "Do you want to save the calibration files with refined stitching parameters? "
                     "Enter 'y' to save the calibration files or any other key to continue without "
                     "saving."
                  << std::endl;
        std::string key;
        std::cin >> key;
        if (key == "y" && !calibInstance.saveCalibFiles(false, filePath)) {
            std::cout << "Error: Failed to save calibration files! Please ensure the target folder "
                         "exists at "
                      << filePath << " and you have write permissions." << std::endl;
            ifExit = true;
            return;
        }
    }

    // Save the stitched files to the specified file path
    if (!calibInstance.saveStitchFilesForZParallel(filePath)) {
        std::cout << "Error: Failed to save Z-parallel stitching files! Please ensure the target "
                     "folder exists at "
                  << filePath << " and you have write permissions." << std::endl;
        ifExit = true;
        return;
    }

    // Get and save stitched point cloud
    ProfileBatch::TexturedPointCloud pointCloud(0);
    if (const auto errorStatus = calibInstance.getStitchedPointCloud(pointCloud);
        !errorStatus.isOK()) {
        Util::printError(errorStatus);
        ifExit = true;
        return;
    }
    ProfileBatch::saveTexturedPointCloud(pointCloud, FileFormat::PLY,
                                         filePath + "/StitchedPointCloud");

    std::cout << "Stitch completed successfully. Stitched files saved in: " << filePath
              << std::endl;
    std::cout << "Save process finished." << std::endl;

    // Prompt the user to continue with image fusion or end the calibration
    std::cout << "\nImage fusion can only be performed effectively if the profilers' z-axis are "
                 "parallel and they are not in a bi-directional (opposing) setup. "
                 "Would you like to proceed with image fusion? Enter 'y' to continue or any other "
                 "key to end the calibration."
              << std::endl;
    std::string key;
    std::cin >> key;

    // Check if the user wants to proceed with image fusion
    if (key != "y") {
        ifExit = true;
        return;
    }

    FusionResult fusionResult;

    // Perform image fusion for Z-parallel calibration
    auto fusionStatus = calibInstance.imageFusionForZParallel(fusionResult);

    /* In the "handleStitchMultiMajorDevice" function, consider the following scenario:
     * - Profiler 0 and Profiler 1 are placed side-by-side,
     * - Profiler 2 and Profiler 3 are also placed side-by-side,
     * - Profiler 0 and Profiler 2 are positioned opposite each other.
     * In this case, only the images from Profiler 0 and Profiler 1, as well as Profiler 2 and
     * Profiler 3, can be fused. If all profilers are arranged side-by-side or in opposite
     * positions, then all images can be fused.
     */
    //// Fuse images of profiler 2 and 3
    // auto fusionStatus = calibInstance.imageFusionForZParallel(
    //     stitchResults.minorStitchResults[1].stitchResultImage,
    //     { stitchResults.minorStitchResults[2] }, stitchResults.minorStitchResults[1].bias,
    //     fusionResult);

    // Check if fusion was successful
    if (!fusionStatus.isOK()) {
        Util::printError(fusionStatus);
        ifExit = true;
        return;
    }

    // Save the fused depth image to the specified file path
    std::cout << "Fusion Success." << std::endl;
    std::string fusionFilePath = filePath + "/fusionResult.tiff";
    cv::imwrite(fusionFilePath, fusionResult.combinedImage.depth);
    ifExit = false;
}
} // namespace

void handleCalibration(bool& ifExit);
void handleStitchSingleMajorDevice(bool& ifExit);
void handleStitchMultiMajorDevice(bool& ifExit);

int main()
{
    bool ifExit = false;
    do {
        ifExit = false;
        switch (Util::enterCommand()) {
        case Util::CommandType::Calibration:
            handleCalibration(ifExit);
            break;
        case Util::CommandType::StitchSingleMajorDevice:
            handleStitchSingleMajorDevice(ifExit);
            break;
        case Util::CommandType::StitchMultiMajorDevice:
            handleStitchMultiMajorDevice(ifExit);
            break;
        case Util::CommandType::Unknown:
            std::cout << "Error: Unknown command" << std::endl;
            std::cout << "Do you want to exit? Enter 'y' to quit or any other key to continue : ";
            char choice;
            std::cin >> choice;
            if (choice == 'y' || choice == 'Y') {
                ifExit = true;
            }
            break;
        }
    } while (!ifExit);
    return 0;
}

void handleCalibration(bool& ifExit)
{
    // Set the target size
    TargetSize targetSize;
    bool continueProcess = false;

    while (!continueProcess) {
        std::cout << "\nEnter the target top length: " << std::endl;
        targetSize.targetTopLength = Util::getInputNumber<double>();

        std::cout << "\nEnter the target bottom length: " << std::endl;
        targetSize.targetBottomLength = Util::getInputNumber<double>();

        std::cout << "\nEnter the target height: " << std::endl;
        targetSize.targetHeight = Util::getInputNumber<double>();

        // Show the input values for confirmation
        std::cout << "\nYou entered the following values:"
                  << "\nTop Length: " << targetSize.targetTopLength
                  << "\nBottom Length: " << targetSize.targetBottomLength
                  << "\nHeight: " << targetSize.targetHeight << std::endl;

        // Ask user if they want to continue or reinput
        std::cout << "\nContinue with the process? (y to continue, any other key to reinput): ";
        char choice;
        std::cin >> choice;
        continueProcess = (choice == 'y' || choice == 'Y');
    }

    // Choose the device for calibration
    std::vector<mmind::eye::Profiler> profilerList =
        Util::findAndConnectMultiProfilerForCalibration();

    if (profilerList.empty()) {
        std::cout << "No profilers connected." << std::endl;
        ifExit = true;
        return;
    }

    // Get the major device Info by retrieving user input
    std::cout << "\nGet the major deviceInfo" << std::endl;
    DeviceInfo majorDeviceInfo = Util::getDeviceInfo(profilerList[0]);
    std::vector<DeviceInfo> minorDeviceInfos;

    // Get the minor device Infos by retrieving user input
    for (int i = 1; i < profilerList.size(); i++) {
        std::cout << "\nGet the minor deviceInfo" << std::endl;
        minorDeviceInfos.push_back(Util::getDeviceInfo(profilerList[i]));
    }

    // Set the target poses
    std::vector<TargetPose> targetsPoses;
    for (int i = 0; i < minorDeviceInfos.size(); i++) {
        TargetPose targetPose;
        bool continuePoseProcess = false;
        while (!continuePoseProcess) {
            std::cout << "\nEnter the pose between targets major and minor-" << i << ":"
                      << std::endl;
            std::cout << "\nEnter the distance between targets: " << std::endl;
            targetPose.translateDistance = Util::getInputNumber<double>();

            std::cout << "\nEnter the rotation angle between targets: " << std::endl;
            targetPose.rotateAngleInDegree = Util::getInputNumber<double>();

            std::cout << "\nEnter the rotation radius between targets: " << std::endl;
            targetPose.rotateRadius = Util::getInputNumber<double>();

            // Show the input values for confirmation
            std::cout << "\nYou entered the following values:"
                      << "\nDistance: " << targetPose.translateDistance
                      << "\nRotation Angle: " << targetPose.rotateAngleInDegree
                      << "\nRotation Radius: " << targetPose.rotateRadius << std::endl;

            // Ask user if they want to continue or reinput
            std::cout << "\nContinue with the process? (y to continue, any other key to reinput): ";
            char choice;
            std::cin >> choice;
            continuePoseProcess = (choice == 'y' || choice == 'Y');
        }

        // Set the calib mode
        ProfilerCalibrationMode calibMode = Util::inputCalibType();

        // Set the transform axis
        Util::TargetTransformAxis transformAxis = Util::inputTargetTransformAxis();

        // Set the translation/rotation axis parameters based on the calibration mode type
        targetPose.mode = calibMode;

        switch (calibMode) {
        case ProfilerCalibrationMode::Wide:
            targetPose.translateAxis = TargetTranslateAxis(static_cast<int>(transformAxis));
            targetPose.rotateAxis = TargetRotateAxis::NullAxis;
            break;

        case ProfilerCalibrationMode::Angle:
            targetPose.rotateAxis = TargetRotateAxis(static_cast<int>(transformAxis));
            targetPose.translateAxis = TargetTranslateAxis::NullAxis;
            break;
        }
        targetsPoses.push_back(targetPose);
    }

    std::cout << "\n++++++++++++++++++++++Start calibration+++++++++++++++++++++++++" << std::endl;

    ProfilerImage majorImage;
    std::vector<cv::Mat> minorDepths;
    std::vector<ProfilerImage> minorImages;

    // Get the major profiler info
    mmind::eye::ProfilerInfo majorProfilerInfo;
    auto status = profilerList[0].getProfilerInfo(majorProfilerInfo);
    if (!status.isOK()) {
        showError(status);
        ifExit = true;
        return;
    }

    // Start to capture images for calibration
    if (!captureImages(profilerList, majorImage, minorDepths, minorImages)) {
        ifExit = true;
        return;
    }

    // Set the params for calibration
    Calibration calibInstance(majorProfilerInfo.model, majorDeviceInfo, minorDeviceInfos,
                              targetSize, targetsPoses);

    // Start to calibrate.
    std::vector<CalibResult> calibResults;
    auto errorStatus =
        calibInstance.calculateCalibration(majorImage.depth, minorDepths, calibResults);

    if (!errorStatus.isOK()) {
        Util::printError(errorStatus);
        ifExit = true;
        return;
    }

    std::cout << "\nCalibration completed successfully." << std::endl;

    std::string filePath;
    std::cout << "Enter the path to save the calibration files (please use English characters for "
                 "the path): ";
    std::cin >> filePath;

    if (!calibInstance.saveCalibFiles(true, filePath)) {
        std::cout
            << "Error: Failed to save calibration files! Please ensure the target folder exists at "
            << filePath << " and you have write permissions." << std::endl;
        ifExit = true;
        return;
    }

    std::cout << "Files saved successfully. Path: " << filePath << std::endl;

    std::cout << "\nWould you like to proceed with image stitching? Enter 'y' to continue or any "
                 "other key to end the calibration."
              << std::endl;

    std::string key;
    std::cin >> key;
    if (key == "y")
        stitchAndFuseImages(calibInstance, majorImage, minorImages, calibResults, filePath, ifExit);
    ifExit = false;
}

void handleStitchSingleMajorDevice(bool& ifExit)
{
    // Choose the device for stitching.
    std::vector<mmind::eye::Profiler> profilerList =
        Util::findAndConnectMultiProfilerForCalibration();
    ProfilerImage majorImage;
    std::vector<cv::Mat> minorDepths;
    std::vector<ProfilerImage> minorImages;

    // Capture images for stitching.
    if (!captureImages(profilerList, majorImage, minorDepths, minorImages)) {
        ifExit = true;
        return;
    }

    // Initialize calibration instance and get the file path for calibration properties
    Calibration calibInstance;
    std::string filePath;
    std::cout << "Enter the calibration file path (please use English characters for the path): ";
    std::cin >> filePath;

    // Load calibration properties from the specified file path
    auto loadErrorStatus = calibInstance.loadCalibProperties(filePath);

    // Check if calibration properties were loaded successfully
    if (!loadErrorStatus.isOK()) {
        Util::printError(loadErrorStatus);
        return;
    }

    // Retrieve current calibration results
    const auto calibResults = calibInstance.getCurrentCalibResults();
    stitchAndFuseImages(calibInstance, majorImage, minorImages, calibResults, filePath, ifExit);
}

void handleStitchMultiMajorDevice(bool& ifExit)
{
    // Initialize calibration instances for different profiler groups
    Calibration calibInstance01, calibInstance02, calibInstance23;

    // Load calibration properties for each profiler group

    // "CalibInstance01" means major profiler is profiler 0, minor profiler is profiler 1
    bool success01 = Util::loadCalibration(
        calibInstance01, "Enter the calibration file path (camera 0 and camera 1): ");
    // "CalibInstance02" means major profiler is profiler 0, minor profiler is profiler 2
    bool success02 = Util::loadCalibration(
        calibInstance02, "Enter the calibration file path (camera 0 and camera 2): ");
    // "CalibInstance23" means major profiler is profiler 2, minor profiler is profiler 3
    bool success23 = Util::loadCalibration(
        calibInstance23, "Enter the calibration file path (camera 2 and camera 3): ");

    if (success01 && success02 && success23) {
        std::cout << "All calibration files loaded successfully!" << std::endl;
    } else {
        if (!success01)
            std::cerr << "Failed to load calibration file for camera 0 and camera 1." << std::endl;
        if (!success02)
            std::cerr << "Failed to load calibration file for camera 0 and camera 2." << std::endl;
        if (!success23)
            std::cerr << "Failed to load calibration file for camera 2 and camera 3." << std::endl;
        std::cerr << "Please check the file paths and try again." << std::endl;
        ifExit = true;
        return;
    }

    // Retrieve calibration results for each profiler group
    const CalibResult calibResults01 = calibInstance01.getCurrentCalibResults()[0];
    const CalibResult calibResults02 = calibInstance02.getCurrentCalibResults()[0];
    const CalibResult calibResults23 = calibInstance23.getCurrentCalibResults()[0];

    // Create new calibration results for profiler group contain profiler 0,1,2,3, the major
    // profiler of new group is profiler 0
    std::vector<CalibResult> calibResults;
    calibResults.push_back(calibResults01);
    calibResults.push_back(calibResults02);

    CalibResult calibResults03(calibResults23);

    // The relevant settings of the major profiler will adopt the settings of profiler 0
    calibResults03.params.majorMoveDirVec = calibResults01.params.majorMoveDirVec;

    // The transformation matrix between profiler 2 and 3 needs to be converted to the
    // transformation matrix between 0 and 3
    calibResults03.params.matrixRT = Util::computeTransformProduct(calibResults02.params.matrixRT,
                                                                   calibResults23.params.matrixRT);

    // calibResults23 transform to calibResults03
    calibResults.push_back(calibResults03);

    // Collect minor device information and target poses from calibration instances
    std::vector<DeviceInfo> minorDeviceInfos;
    std::vector<TargetPose> targetPoses;
    minorDeviceInfos.push_back(calibInstance01.getMinorDeviceInfos()[0]);
    minorDeviceInfos.push_back(calibInstance02.getMinorDeviceInfos()[0]);
    minorDeviceInfos.push_back(calibInstance23.getMinorDeviceInfos()[0]);

    targetPoses.push_back(calibInstance01.getTargetPoses()[0]);
    targetPoses.push_back(calibInstance02.getTargetPoses()[0]);
    targetPoses.push_back(calibInstance23.getTargetPoses()[0]);

    // Initialize the Calibration instance using the new constructor to reduce code volume
    // Parameters in order:
    // 1. calibInstance01.getCameraModel(): Set the camera model, same as profiler 0
    // 2. calibInstance01.getMajorDeviceInfo(): Set the major device info, same as profiler 0
    // 3. minorDeviceInfos: Set the minor device infos, containing profiler 1, 2, 3
    // 4. calibInstance01.getTargetSize(): Set the calibration target size, same as profiler 0
    // 5. targetPoses: Set the poses of the calibration target
    Calibration calibInstance(calibInstance01.getCameraModel(),
                              calibInstance01.getMajorDeviceInfo(), minorDeviceInfos,
                              calibInstance01.getTargetSize(), targetPoses);

    // Choose the device for stitching.
    std::vector<mmind::eye::Profiler> profilerList =
        Util::findAndConnectMultiProfilerForCalibration();

    // Variables to store the major image and minor images for stitching
    ProfilerImage majorImage;
    std::vector<cv::Mat> minorDepths;
    std::vector<ProfilerImage> minorImages;

    // Capture images from each profiler device
    if (!captureImages(profilerList, majorImage, minorDepths, minorImages)) {
        ifExit = true;
        return;
    }

    std::string filePath;
    std::cout << "Enter the path you want to save the stitch result (please use English characters "
                 "for the path): ";
    std::cin >> filePath;
    stitchAndFuseImages(calibInstance, majorImage, minorImages, calibResults, filePath, ifExit);
}
