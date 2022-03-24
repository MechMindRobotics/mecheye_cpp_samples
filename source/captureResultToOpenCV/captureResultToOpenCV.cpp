#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "MechEyeApi.h"

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
    status = device.getDeviceInfo(deviceInfo);
    printDeviceInfo(deviceInfo);

    mmind::api::ColorMap color;
    showError(device.captureColorMap(color));
    const std::string colorFile = "colorMap.png";
    cv::Mat color8UC3 = cv::Mat(color.height(), color.width(), CV_8UC3, color.data());
    cv::imwrite(colorFile, color8UC3);
    std::cout << "Capture and save color image : " << colorFile << std::endl;

    mmind::api::DepthMap depth;
    showError(device.captureDepthMap(depth));
    const std::string depthFile = "depthMap.png";
    cv::Mat depth8U;
    cv::Mat depth32F = cv::Mat(depth.height(), depth.width(), CV_32FC1, depth.data());
    double minDepth, maxDepth;
    cv::minMaxLoc(depth32F, &minDepth, &maxDepth);
    depth32F.convertTo(depth8U, CV_8UC1, 255.0 / (maxDepth));
    cv::imwrite(depthFile, depth8U);
    std::cout << "Capture and save depth image : " << depthFile << std::endl;

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

bool isNumber(const std::string& str)
{
    for (auto it = str.cbegin(); it != str.cend(); ++it) {
        if (*it < '0' || *it > '9')
            return false;
    }
    return true;
}
