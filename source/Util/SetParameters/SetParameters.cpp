#include "MechEyeApi.h"
#include "SampleUtil.h"
#include <iostream>

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    mmind::api::MechEyeDeviceInfo deviceInfo;
    showError(device.getDeviceInfo(deviceInfo));
    printDeviceInfo(deviceInfo);

    std::vector<std::string> userSets;
    showError(device.getAllUserSets(userSets));

    std::cout << "All user sets : ";
    for (size_t i = 0; i < userSets.size(); i++)
        std::cout << userSets[i] << "  ";
    std::cout << std::endl;

    std::string currentUserSet;
    showError(device.getCurrentUserSet(currentUserSet));
    std::cout << "Current user set : " << currentUserSet << std::endl;

    showError(device.setCurrentUserSet(userSets.front()));
    std::cout << "Set \"" << userSets.front() << "\" as the current user set." << std::endl
              << std::endl;

    showError(device.setScan3DExposure(std::vector<double>{5, 10}));

    std::vector<double> exposureSequence;
    showError(device.getScan3DExposure(exposureSequence));

    std::cout << "The 3D scanning exposure multiplier : " << exposureSequence.size() << "."
              << std::endl;
    for (size_t i = 0; i < exposureSequence.size(); i++)
        std::cout << "3D scanning exposure time " << i + 1 << " : " << exposureSequence[i] << " ms."
                  << std::endl;

    showError(device.setDepthRange(mmind::api::DepthRange(100, 1000)));
    mmind::api::DepthRange depthRange;
    showError(device.getDepthRange(depthRange));
    std::cout << "3D scanning depth Lower Limit : " << depthRange.lower
              << " mm, depth upper limit : " << depthRange.upper << " mm." << std::endl;

    showError(device.setScan3DROI(mmind::api::ROI(0, 0, 500, 500)));
    mmind::api::ROI scan3dRoi;
    showError(device.getScan3DROI(scan3dRoi));
    std::cout << "3D scanning ROI topLeftX : " << scan3dRoi.x << ", topLeftY : " << scan3dRoi.y
              << ", width : " << scan3dRoi.width << ", height : " << scan3dRoi.height << std::endl;

    showError(
        device.setScan2DExposureMode(mmind::api::Scanning2DSettings::Scan2DExposureMode::Timed));
    showError(device.setScan2DExposureTime(100));

    mmind::api::Scanning2DSettings::Scan2DExposureMode exposureMode2D;
    double scan2DExposureTime;
    showError(device.getScan2DExposureMode(exposureMode2D));
    showError(device.getScan2DExposureTime(scan2DExposureTime));
    std::cout << "2D scanning exposure mode enum : " << static_cast<int>(exposureMode2D)
              << ", exposure time : " << scan2DExposureTime << " ms." << std::endl;

    showError(device.setCloudSmoothMode(
        mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Normal));
    showError(device.setCloudOutlierFilterMode(
        mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Normal));

    mmind::api::PointCloudProcessingSettings::CloudSmoothMode cloudSmoothMode;
    mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode cloudOutlierFilterMode;
    showError(device.getCloudSmoothMode(cloudSmoothMode));
    showError(device.getCloudOutlierFilterMode(cloudOutlierFilterMode));

    std::cout << "Cloud smooth mode enum : " << static_cast<int>(cloudSmoothMode)
              << ", cloud outlier filter mode enum : " << static_cast<int>(cloudOutlierFilterMode)
              << std::endl;

    showError(device.saveAllSettingsToUserSets());
    std::cout << "save all parammeters to current user set." << std::endl;

    device.disconnect();
    std::cout << "Disconnect Mech-Eye Success." << std::endl;
    return 0;
}
