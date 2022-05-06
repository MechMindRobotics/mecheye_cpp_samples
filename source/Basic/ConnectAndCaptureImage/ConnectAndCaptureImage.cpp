#include "MechEyeApi.h"
#include "SampleUtil.h"
#include <iostream>

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

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
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
