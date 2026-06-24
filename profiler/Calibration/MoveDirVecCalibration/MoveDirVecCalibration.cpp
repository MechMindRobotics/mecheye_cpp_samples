#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "../SingleProfilersCalibration.h"

using namespace mmind::eye;
using Calibration = mmind::eye::ProfilerCalibrationInterfaces;

/**
 * @brief Entry of single profiler movement direction calibration and correction demo.
 *
 * This demo demonstrates the complete workflow of:
 * 1) Capturing depth images using a single line-scan profiler
 * 2) Calibrating the movement direction vector of the profiler
 * 3) Correcting the scanning direction to obtain a rectified depth image
 * 4) Removing pixel offset to align the result back to the camera coordinate system
 *
 * The final aligned depth image can be directly used for stitching or further measurement.
 */
int main()
{
    // Step 1. Input target parameters.
    TargetSize targetSize;
    while (true) {
        std::cout << "\nPlease input target parameters:\n";
        std::cout << "Target top length: ";
        targetSize.targetTopLength = getInputNumber<float>();

        std::cout << "Target bottom length: ";
        targetSize.targetBottomLength = getInputNumber<float>();

        std::cout << "Target height: ";
        targetSize.targetHeight = getInputNumber<float>();

        std::cout << "\nConfirm target parameters:\n"
                  << "Top Length    : " << targetSize.targetTopLength << "\n"
                  << "Bottom Length : " << targetSize.targetBottomLength << "\n"
                  << "Height        : " << targetSize.targetHeight << "\n"
                  << "Continue? (y/n): ";

        char confirm;
        std::cin >> confirm;
        if (confirm == 'y' || confirm == 'Y') {
            break;
        }
    }

    // Step 2. Connect profiler and acquire device information.
    auto profilerOpt = findAndConnectProfilerForCalibration();
    if (!profilerOpt) {
        std::cout << "Failed to connect profiler.\n";
        return -1;
    }
    auto profiler = profilerOpt.value();

    // Get the device Info by retrieving user input.
    std::cout << "\nGet the major deviceInfo" << std::endl;
    DeviceInfo profilerDeviceInfo = getDeviceInfo(profiler);

    // Get the profiler info
    mmind::eye::ProfilerInfo profilerInfo;
    auto status = profiler.getProfilerInfo(profilerInfo);
    if (!status.isOK()) {
        showError(status);
        return -1;
    }

    /**************************************************************************
     * Step 3. Capture images for movement direction calibration
     *
     * Two depth images are required:
     * - depth1: captured during the first scan (A -> B)
     * - depth2: captured during the second scan (C -> D)
     **************************************************************************/
    std::vector<ProfilerImage> profilerImages;
    // Start to capture images for calibration.
    if (!captureImages(profiler, profilerImages) || profilerImages.size() < 2) {
        std::cerr << "Failed to capture sufficient images for calibration.\n";
        return -1;
    }

    // Step 4. Configure calibration parameters.
    std::vector<TargetPose> targetsPoses;
    TargetPose targetPose;

    bool continuePoseProcess = false;
    while (!continuePoseProcess) {
        std::cout << "\nEnter the distance between targets: " << std::endl;
        targetPose.translateDistance = getInputNumber<float>();

        std::cout << "\nEnter the rotation angle between targets: " << std::endl;
        targetPose.rotateAngleInDegree = getInputNumber<float>();

        std::cout << "\nEnter the rotation radius between targets: " << std::endl;
        targetPose.rotateRadius = getInputNumber<float>();

        // Show the input values for confirmation.
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
    // Set the calib mode.
    ProfilerCalibrationMode calibMode = inputCalibType();

    // Set the transform axis.
    TargetTransformAxis transformAxis = inputTargetTransformAxis();

    // Set the translation/rotation axis parameters based on the calibration mode type.
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

    ProfilerMovementAxis moveAxis = inputMovementAxis();

    std::cout << "\n+++++++Starting movement direction calibration++++++" << std::endl;

    // Set the params for calibration.
    Calibration calibInstance(profilerInfo.model, profilerDeviceInfo, targetSize, targetsPoses);

    MoveDirCalibResult calibResult;

    auto errorStatus = calibInstance.calibrateSingleProfilerMoveDirection(
        profilerImages[0].depth, profilerImages[1].depth, moveAxis, calibResult);

    if (!errorStatus.isOK()) {
        printError(errorStatus);
        return -1;
    }

    std::cout << "\nCalibration completed successfully." << std::endl;
    std::cout << "Movement direct vector is : {" << calibResult.moveDirVec.x << ","
              << calibResult.moveDirVec.y << "," << calibResult.moveDirVec.z << "}" << std::endl;

    /**************************************************************************
     * Step 6. Correct scanning direction (rotation only)
     *
     * This step rectifies the scanning direction so that the scan axis aligns
     * with the camera coordinate system. The output image is direction-corrected
     * but may contain a pixel offset relative to the camera origin.
     **************************************************************************/
    Calibration::CorrectedResult correctedResult;

    errorStatus = calibInstance.correctProfilerMoveDir(calibResult.moveDirVec, profilerImages[0],
                                                       correctedResult);

    if (!errorStatus.isOK()) {
        printError(errorStatus);
        return -1;
    }

    std::cout << "\nDirection correction completed." << std::endl;
    std::cout << "Pixel offset to camera origin: [" << correctedResult.pixelBiasToCamCoord.x << ", "
              << correctedResult.pixelBiasToCamCoord.y << "] (pixels)\n";

    /**************************************************************************
     * Step 7. Remove pixel offset (align back to camera coordinate system)
     *
     * The correction step rotates the scan around the camera origin, which
     * introduces a pixel offset. This offset must be removed before the result
     * can be used for stitching or multi-frame fusion.
     **************************************************************************/
    cv::Mat alignedDepth;
    cv::Mat translationMat =
        (cv::Mat_<double>(2, 3) << 1, 0, -correctedResult.pixelBiasToCamCoord.x, 0, 1,
         -correctedResult.pixelBiasToCamCoord.y);

    cv::warpAffine(correctedResult.depth, alignedDepth, translationMat,
                   correctedResult.depth.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT,
                   cv::Scalar(std::numeric_limits<float>::quiet_NaN()));

    // Save results for verification.
    cv::imwrite("01_raw_depth.tiff", profilerImages[0].depth);
    cv::imwrite("02_corrected_depth_with_offset.tiff", correctedResult.depth);
    cv::imwrite("03_aligned_depth_for_stitching.tiff", alignedDepth);

    std::cout << "\nDemo completed successfully.\n";
    std::cout << "Saved results:\n"
              << " - 01_raw_depth.tiff\n"
              << " - 02_corrected_depth_with_offset.tiff\n"
              << " - 03_aligned_depth_for_stitching.tiff\n";

    /**************************************************************************
     * // Pixel-to-physical resolution (example: mm per pixel)
     *	constexpr double kPixelResolutionX = 0.0235; // mm / pixel
     *	constexpr double kPixelResolutionY = 0.0235;
     *
     *	// Convert pixel bias to physical offset (mm)
     *	double offsetX_mm = correctedResult.pixelBiasToCamCoord.x * kPixelResolutionX;
     *	double offsetY_mm = correctedResult.pixelBiasToCamCoord.y * kPixelResolutionY;
     *	// Use these values in camera/world coordinate calibration
     **************************************************************************/

    return 0;
}