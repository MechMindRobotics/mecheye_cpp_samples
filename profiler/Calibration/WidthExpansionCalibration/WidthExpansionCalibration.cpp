#include <opencv2/imgcodecs.hpp>
#include "../SingleProfilersCalibration.h"

using namespace mmind::eye;
using Calibration = mmind::eye::ProfilerCalibrationInterfaces;

int main()
{
    // Target geometry input.
    TargetSize targetSize;
    bool confirmed = false;

    while (!confirmed) {
        std::cout << "\nInput target geometry parameters (unit: mm)\n";

        std::cout << "Target top length: ";
        targetSize.targetTopLength = getInputNumber<float>();

        std::cout << "Target bottom length: ";
        targetSize.targetBottomLength = getInputNumber<float>();

        std::cout << "Target height: ";
        targetSize.targetHeight = getInputNumber<float>();

        std::cout << "\nConfirm target geometry:\n"
                  << "  Top Length   : " << targetSize.targetTopLength << "\n"
                  << "  Bottom Length: " << targetSize.targetBottomLength << "\n"
                  << "  Height       : " << targetSize.targetHeight << "\n";

        std::cout << "\nContinue? (y/Y to confirm, others to re-input): ";
        char choice;
        std::cin >> choice;
        confirmed = (choice == 'y' || choice == 'Y');
    }

    // Connect profiler.
    auto profilerOpt = findAndConnectProfilerForCalibration();
    if (!profilerOpt) {
        std::cout << "Error: Failed to connect profiler.\n";
        return -1;
    }
    auto profiler = profilerOpt.value();

    std::cout << "\nRetrieving profiler device information...\n";
    DeviceInfo profilerDeviceInfo = getDeviceInfo(profiler);

    // Target pose input.
    std::vector<TargetPose> targetPoses;
    TargetPose targetPose;
    confirmed = false;

    while (!confirmed) {
        std::cout << "\nInput target motion parameters\n";

        std::cout << "Translation distance (mm): ";
        targetPose.translateDistance = getInputNumber<float>();

        std::cout << "Rotation angle (degree): ";
        targetPose.rotateAngleInDegree = getInputNumber<float>();

        std::cout << "Rotation radius (mm): ";
        targetPose.rotateRadius = getInputNumber<float>();

        std::cout << "\nConfirm target pose:\n"
                  << "  Translation distance: " << targetPose.translateDistance << "\n"
                  << "  Rotation angle      : " << targetPose.rotateAngleInDegree << "\n"
                  << "  Rotation radius     : " << targetPose.rotateRadius << "\n";

        std::cout << "\nContinue? (y/Y to confirm, others to re-input): ";
        char choice;
        std::cin >> choice;
        confirmed = (choice == 'y' || choice == 'Y');
    }

    // Calibration mode setup.
    ProfilerCalibrationMode calibMode = inputCalibType();
    TargetTransformAxis transformAxis = inputTargetTransformAxis();

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

    targetPoses.push_back(targetPose);

    // Width expansion type.
    WidthExpansionType widthExpansionMode = inputWidthExpansionCalibType();

    /* =========================
     * Reference position bias.
     * =========================
     * refPositionBias describes the expected relative spatial offset
     * between the major scan and the minor scan (unit: mm).
     * It is REQUIRED for width expansion calibration.
     */
    std::vector<RefPositionBias> refPositionBiases;

    std::cout << "\n==================== Start Calibration ====================\n";

    // Capture calibration images.
    ProfilerImage majorImage;
    std::vector<ProfilerImage> minorImages;

    ProfilerInfo profilerInfo;
    auto status = profiler.getProfilerInfo(profilerInfo);
    if (!status.isOK()) {
        showError(status);
        return -1;
    }

    if (!captureImages(profiler, minorImages, majorImage)) {
        std::cout << "Error: Failed to capture calibration images.\n";
        return -1;
    }

    for (int i = 0; i < minorImages.size(); i++) {
        RefPositionBias bias;
        bias.groupID = (uint)i;
        std::cout << "\nInput reference position bias X (mm): " << std::endl;
        bias.biasMm.x = getInputNumber<float>();
        std::cout << "\nInput reference position bias Y (mm): " << std::endl;
        bias.biasMm.y = getInputNumber<float>();
        std::cout << "\nInput reference position bias Z (mm): " << std::endl;
        bias.biasMm.z = getInputNumber<float>();
        refPositionBiases.push_back(bias);
    }

    // Run calibration.
    std::vector<StitchParams> calibResults;
    Calibration calibInstance(profilerInfo.model, profilerDeviceInfo, targetSize, targetPoses);

    auto errorStatus = calibInstance.calibrateSingleProfilerWidthExpansion(
        majorImage.depth, minorImages[0].depth, refPositionBiases, calibResults);

    if (!errorStatus.isOK()) {
        printError(errorStatus);
        return -1;
    }

    std::cout << "\nCalibration completed successfully.\n";

    // Save calibration files.
    std::string savePath;
    std::cout << "\nInput path to save calibration files: ";
    std::cin >> savePath;

    if (!calibInstance.saveCalibFiles(true, savePath)) {
        std::cout << "Error: Failed to save calibration files.\n";
        return -1;
    }

    std::cout << "Calibration files saved to: " << savePath << "\n";

    // Optional stitching verification.
    std::cout << "\nProceed with stitching verification? (y/Y to continue): ";
    std::string option;
    std::cin >> option;

    if (option == "y" || option == "Y") {
        FusionResult fusionResult;

        ImageInfo majorImageInfo;
        majorImageInfo.profilerImage = majorImage;

        std::vector<ImageInfo> minorImageInfos;
        for (size_t i = 0; i < minorImages.size(); ++i) {
            ImageInfo info;
            info.groupID = static_cast<int>(i);
            info.profilerImage = minorImages[i];
            minorImageInfos.push_back(info);
        }

        errorStatus = calibInstance.stitchProfilerWidthExpansion(
            majorImageInfo, minorImageInfos, widthExpansionMode, fusionResult, calibResults, true);

        if (!errorStatus.isOK()) {
            printError(errorStatus);
            return -1;
        }

        std::cout << "Stitching verification succeeded.\n";

        std::cout << "\nSave refined calibration files? (y/Y to save): ";
        std::cin >> option;

        if ((option == "y" || option == "Y") && !calibInstance.saveCalibFiles(false, savePath)) {
            std::cout << "Error: Failed to save refined calibration files.\n";
            return -1;
        }

        std::string fusionImagePath = savePath + "/fusionResult.tiff";
        cv::imwrite(fusionImagePath, fusionResult.combinedImage.depth);

        std::cout << "Fusion image saved to: " << fusionImagePath << "\n";
    }

    std::cout << "\nProcess finished successfully.\n";
    return 0;
}
