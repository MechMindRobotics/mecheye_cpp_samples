#include "MechEyeApi.h"
#include <iostream>

bool isNumber(const std::string& str);
void showError(const mmind::api::ErrorStatus& status);
void printDeviceInfo(const mmind::api::MechEyeDeviceInfo& deviceInfo);

int main()
{
    std::cout << "Find Mech-Eye device..." << std::endl;
    std::vector<mmind::api::MechEyeDeviceInfo> deviceInfoList =
       mmind::api::MechEyeDevice::enumerateMechEyeDeviceList();

    if (deviceInfoList.empty()) {
       std::cout << "No Mech-Eye device found." << std::endl;
       return -1;
    }

    for (int i = 0; i < deviceInfoList.size(); i++) {
       std::cout << "Mech-Eye device index : " << i << std::endl;
       printDeviceInfo(deviceInfoList[i]);
    }

    std::cout << "Please enter the device index you want to connect: ";
    unsigned inputIndex;

    while (1) {
       std::string str;
       std::cin >> str;
       if (isNumber(str) && atoi(str.c_str()) < deviceInfoList.size()) {
           inputIndex = atoi(str.c_str());
           break;
       }
       std::cout << "Input invalid! Please enter the device index you want to connect: ";
    }

    mmind::api::ErrorStatus status;
    mmind::api::MechEyeDevice device;
    status = device.connect(deviceInfoList[inputIndex]);

    if (!status.isOK()) {
        showError(status);
        return -1;
    }

    std::cout << "Connect Mech-Eye Success." << std::endl;

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

    showError(device.setScan3DExposure(std::vector<double>{0.1, 5, 10}));

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

void showError(const mmind::api::ErrorStatus& status)
{
    if (status.isOK())
        return;
    std::cout << "Error Code : " << status.errorCode
              << ", Error Description: " << status.errorDescription << std::endl;
}

void printDeviceInfo(const mmind::api::MechEyeDeviceInfo& deviceInfo)
{
    std::cout << "Camera Model Name: " << deviceInfo.model << std::endl;
    std::cout << "Camera ID:         " << deviceInfo.id << std::endl;
    std::cout << "Hardware Version:  "
              << "V" << deviceInfo.hardwareVersion << std::endl;
    std::cout << "Firmware Version:  "
              << "V" << deviceInfo.firmwareVersion << std::endl;
    std::cout << std::endl;
}

bool isNumber(const std::string& str)
{
    for (auto it = str.cbegin(); it != str.cend(); ++it) {
        if (*it < '0' || *it > '9')
            return false;
    }
    return true;
}
