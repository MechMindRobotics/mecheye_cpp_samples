#include "MechEyeApi.h"
#include <iostream>

bool isNumber(const std::string& str);
void showError(const mmind::api::ErrorStatus& status);
void printDeviceInfo(const mmind::api::MechEyeDeviceInfo& deviceInfo);
void printCalibParams(const mmind::api::DeviceIntri& deviceIntri);
void printDeviceResolution(const mmind::api::DeviceResolution& deviceResolution);

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

    mmind::api::DeviceIntri deviceIntri;
    showError(device.getDeviceIntri(deviceIntri));
    printCalibParams(deviceIntri);

    mmind::api::DeviceResolution deviceResolution;
    showError(device.getDeviceResolution(deviceResolution));
    printDeviceResolution(deviceResolution);

    mmind::api::ColorMap colorMap;
    unsigned row = 0;
    unsigned col = 0;
    showError(device.captureColorMap(colorMap));
    std::cout << "Color map size is width: " << colorMap.width() << " height: " << colorMap.height()
              << "." << std::endl;
    try {
        mmind::api::ElementColor colorElem = colorMap.at(row, col);
        std::cout << "Color map element at ( " << row << "," << col
                  << ") is R: " << unsigned(colorElem.r) << " G: " << unsigned(colorElem.g)
                  << " B: " << unsigned(colorElem.b) << "." << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        device.disconnect();
        return 0;
    }

    mmind::api::DepthMap depthMap;
    showError(device.captureDepthMap(depthMap));
    std::cout << "Depth map size is width: " << depthMap.width() << " height: " << depthMap.height()
              << "." << std::endl;
    try {
        mmind::api::ElementDepth depthElem = depthMap.at(row, col);
        std::cout << "Depth map element at ( " << row << "," << col << ") is depth: " << depthElem.d
                  << " mm." << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        device.disconnect();
        return 0;
    }

    mmind::api::PointXYZMap pointXYZMap;
    showError(device.capturePointXYZMap(pointXYZMap));
    std::cout << "Pointcloud map size is width: " << pointXYZMap.width()
              << " height: " << pointXYZMap.height() << "." << std::endl;
    try {
        mmind::api::ElementPointXYZ pointXYZElem = pointXYZMap.at(row, col);
        std::cout << "PointXYZ map element at ( " << row << "," << col
                  << ") is X: " << pointXYZElem.x << " mm  Y: " << pointXYZElem.y
                  << " mm  Z:" << pointXYZElem.z << " mm." << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        device.disconnect();
        return 0;
    }

    device.disconnect();
    std::cout << "Disconnect Mech-Eye Success." << std::endl;
    return 0;
}

void showError(const mmind::api::ErrorStatus& status)
{
    if (status.isOK())
        return;
    std::cout << "Error Code : " << status.errorCode
              << ", Error Description: " << status.errorDescription << "." << std::endl;
}

void printDeviceInfo(const mmind::api::MechEyeDeviceInfo& deviceInfo)
{
    std::cout << "............................" << std::endl;
    std::cout << "Camera Model Name: " << deviceInfo.model << std::endl;
    std::cout << "Camera ID:         " << deviceInfo.id << std::endl;
    std::cout << "Camera IP Address: " << deviceInfo.ipAddress << std::endl;
    std::cout << "Hardware Version:  "
              << "V" << deviceInfo.hardwareVersion << std::endl;
    std::cout << "Firmware Version:  "
              << "V" << deviceInfo.firmwareVersion << std::endl;
    std::cout << "............................" << std::endl;
    std::cout << std::endl;
}

void printDeviceResolution(const mmind::api::DeviceResolution& deviceResolution)
{
    std::cout << "Color Map size : (width : " << deviceResolution.colorMapWidth
              << ", height : " << deviceResolution.colorMapHeight << ")." << std::endl;
    std::cout << "Depth Map size : (width : " << deviceResolution.depthMapWidth
              << ", height : " << deviceResolution.depthMapHeight << ")." << std::endl;
}

void printMatrix(const std::string& name, const double* cameraMatrix)
{
    std::cout << name << ": " << std::endl
              << "    [" << cameraMatrix[0] << ", " << 0 << ", " << cameraMatrix[2] << "]"

              << std::endl
              << "    [" << 0 << ", " << cameraMatrix[1] << ", " << cameraMatrix[3] << "]"

              << std::endl
              << "    [" << 0 << ", " << 0 << ", " << 1 << "]" << std::endl;
    std::cout << std::endl;
}

void printDistCoeffs(const std::string& name, const double* distCoeffs)
{
    std::cout << name << ": " << std::endl
              << "    k1: " << distCoeffs[0] << ", k2: " << distCoeffs[1]
              << ", p1: " << distCoeffs[2] << ", p2: " << distCoeffs[3] << ", k3: " << distCoeffs[4]
              << std::endl;
    std::cout << std::endl;
}

void printCalibParams(const mmind::api::DeviceIntri& deviceIntri)
{
    printMatrix("CameraMatrix", deviceIntri.cameraMatrix);
    printDistCoeffs("CameraDistCoeffs", deviceIntri.distCoeffs);
}

bool isNumber(const std::string& str)
{
    for (auto it = str.cbegin(); it != str.cend(); ++it) {
        if (*it < '0' || *it > '9')
            return false;
    }
    return true;
}
