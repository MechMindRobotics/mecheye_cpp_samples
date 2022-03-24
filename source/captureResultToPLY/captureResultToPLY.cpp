#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "MechEyeApi.h"

bool isNumber(const std::string& str);
void showError(const mmind::api::ErrorStatus& status);
void printDeviceInfo(const mmind::api::MechEyeDeviceInfo& deviceInfo);
void savePLY(const mmind::api::PointXYZMap& pointXYZMap, const std::string& path);
void savePLY(const mmind::api::PointXYZMap& pointXYZMap, const mmind::api::ColorMap& colorMap,
             const std::string& path);

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

    mmind::api::ColorMap colorMap;
    device.captureColorMap(colorMap);

    mmind::api::PointXYZMap pointXYZMap;
    device.capturePointXYZMap(pointXYZMap);

    std::string pointCloudPath = "pointCloudXYZ.ply";
    savePLY(pointXYZMap, pointCloudPath);

    std::string pointCloudColorPath = "pointCloudXYZRGB.ply";
    savePLY(pointXYZMap, colorMap, pointCloudColorPath);

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

void savePLY(const mmind::api::PointXYZMap& pointXYZMap, const std::string& path)
{
    // write pointcloudXYZ data
    pcl::PointCloud<pcl::PointXYZ> pointCloud(pointXYZMap.width(), pointXYZMap.height());
    uint32_t size = pointXYZMap.height() * pointXYZMap.width();
    pointCloud.resize(size);

    for (size_t i = 0; i < size; i++) {
        pointCloud[i].x = pointXYZMap[i].x;
        pointCloud[i].y = pointXYZMap[i].y;
        pointCloud[i].z = pointXYZMap[i].z;
    }

    pcl::PLYWriter writer;
    writer.write(path, pointCloud, true);
    std::cout << "PointCloudXYZ has : " << pointCloud.width * pointCloud.height << " data points."
              << std::endl;

    return;
}

void savePLY(const mmind::api::PointXYZMap& pointXYZMap, const mmind::api::ColorMap& colorMap,
             const std::string& path)
{
    // write pointcloudXYZRGB data
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud(pointXYZMap.width(), pointXYZMap.height());
    uint32_t size = pointXYZMap.height() * pointXYZMap.width();
    pointCloud.resize(size);

    for (size_t i = 0; i < size; i++) {
        pointCloud[i].x = pointXYZMap[i].x;
        pointCloud[i].y = pointXYZMap[i].y;
        pointCloud[i].z = pointXYZMap[i].z;

        pointCloud[i].r = colorMap[i].r;
        pointCloud[i].g = colorMap[i].g;
        pointCloud[i].b = colorMap[i].b;
    }

    pcl::PLYWriter writer;
    writer.write(path, pointCloud, true);
    std::cout << "PointCloudXYZRGB has : " << pointCloud.width * pointCloud.height
              << " data points." << std::endl;

    return;
}

bool isNumber(const std::string& str)
{
    for (auto it = str.cbegin(); it != str.cend(); ++it) {
        if (*it < '0' || *it > '9')
            return false;
    }
    return true;
}
