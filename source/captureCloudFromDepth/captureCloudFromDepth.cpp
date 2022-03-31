#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "MechEyeApi.h"

bool isNumber(const std::string& str);
void showError(const mmind::api::ErrorStatus& status);
void printDeviceInfo(const mmind::api::MechEyeDeviceInfo& deviceInfo);
void savePLY(const mmind::api::DepthMap& depth, const std::string& path, const mmind::api::DeviceIntri& intri);
void savePLY(const mmind::api::DepthMap& depth, const mmind::api::ColorMap& color,
             const std::string& path, const mmind::api::DeviceIntri& intri);

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

    mmind::api::DepthMap depth;
    showError(device.captureDepthMap(depth));

    mmind::api::DeviceIntri deviceIntri;
    showError(device.getDeviceIntri(deviceIntri));

    std::string pointCloudPath = "pointCloudXYZ.ply";
    savePLY(depth, pointCloudPath, deviceIntri);

    std::string pointCloudColorPath = "pointCloudXYZRGB.ply";
    savePLY(depth, color, pointCloudColorPath, deviceIntri);

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

void savePLY(const mmind::api::DepthMap& depth, const std::string& path, const mmind::api::DeviceIntri& intri)
{
    // write pointcloudXYZ data
    pcl::PointCloud<pcl::PointXYZ> pointCloud(depth.width(), depth.height());
    uint32_t size = depth.height() * depth.width();
    pointCloud.resize(size);

    for (int m = 0; m < depth.height(); ++m)
        for (int n = 0; n < depth.width(); ++n) {
            float d;
            try{
                d = depth.at(m, n).d;
            } catch (const std::exception &e) {
                std::cout << "Exception: " << e.what() << std::endl;
                return;
            }
            pointCloud.at(n, m).z = d;
            pointCloud.at(n, m).x = (n - intri.cameraMatrix[2]) * d / intri.cameraMatrix[0];
            pointCloud.at(n, m).y = (m - intri.cameraMatrix[3]) * d / intri.cameraMatrix[1];
        }

    pcl::PLYWriter writer;
    writer.write(path, pointCloud, true);
    std::cout << "PointCloudXYZ has : " << pointCloud.width * pointCloud.height << " data points."
              << std::endl;

    return;
}

void savePLY(const mmind::api::DepthMap& depth, const mmind::api::ColorMap& color,
             const std::string& path, const mmind::api::DeviceIntri& intri)
{
    // write pointcloudXYZRGB data
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud(depth.width(), depth.height());
    uint32_t size = depth.height() * depth.width();
    pointCloud.resize(size);

    for (int m = 0; m < depth.height(); ++m)
        for (int n = 0; n < depth.width(); ++n) {
            float d;
            uint8_t r, g, b;
            try {
                d = depth.at(m, n).d;
                r = color.at(m, n).r;
                g = color.at(m, n).g;
                b = color.at(m, n).b;
            } catch (const std::exception &e) {
                std::cout << "Exception: " << e.what() << std::endl;
                return;
            }
            pointCloud.at(n, m).z = d;
            pointCloud.at(n, m).x = (n - intri.cameraMatrix[2]) * d / intri.cameraMatrix[0];
            pointCloud.at(n, m).y = (m - intri.cameraMatrix[3]) * d / intri.cameraMatrix[1];

            pointCloud.at(n, m).r = r;
            pointCloud.at(n, m).g = g;
            pointCloud.at(n, m).b = b;
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
