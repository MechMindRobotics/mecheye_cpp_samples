#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <set>
#include <vector>

#include <opencv2/highgui/highgui.hpp>

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include "PclUtil.h"

void capture(mmind::api::MechEyeDevice& device, std::mutex& m);

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

    std::string str;
    std::set<unsigned> indices;

    while (1) {
        std::cout << "Please enter the device index you want to connect: " << std::endl;
        std::cout << "Enter a c to terminate adding devices" << std::endl;
        unsigned inputIndex;

        std::cin >> str;
        if (str == "c")
            break;
        if (isNumber(str) && atoi(str.c_str()) < deviceInfoList.size())
            inputIndex = atoi(str.c_str());
        else {
            std::cout << "Input invalid! Please enter the device index you want to connect: ";
            continue;
        }

        indices.emplace(inputIndex);
    }

    mmind::api::MechEyeDevice* devices = new mmind::api::MechEyeDevice[indices.size()];

    std::set<unsigned>::iterator it = indices.begin();
    std::vector<std::future<void>> container;
    std::mutex m;
    for (int i = 0; i < indices.size() && it != indices.end(); ++i, ++it) {
        showError(devices[i].connect(deviceInfoList[*it]));
        container.emplace_back(
            std::async(std::launch::async, capture, std::ref(devices[i]), std::ref(m)));
    }

    for (int i = 0; i < indices.size(); ++i) {
        container[i].get();
        devices[i].disconnect();
    }

    delete[] devices;
    return 0;
}

void capture(mmind::api::MechEyeDevice& device, std::mutex& m)
{
    mmind::api::MechEyeDeviceInfo info;
    showError(device.getDeviceInfo(info));
    std::string id = info.id;

    mmind::api::ColorMap color;
    showError(device.captureColorMap(color));
    const std::string colorFile = "colorMap_" + id + ".png";
    cv::Mat color8UC3 = cv::Mat(color.height(), color.width(), CV_8UC3, color.data());

    mmind::api::DepthMap depth;
    showError(device.captureDepthMap(depth));
    const std::string depthFile = "depthMap_" + id + ".png";
    cv::Mat depth8U;
    cv::Mat depth32F = cv::Mat(depth.height(), depth.width(), CV_32FC1, depth.data());
    double minDepth, maxDepth;
    cv::minMaxLoc(depth32F, &minDepth, &maxDepth);
    depth32F.convertTo(depth8U, CV_8UC1, 255.0 / (maxDepth));

    mmind::api::PointXYZMap pointXYZMap;
    device.capturePointXYZMap(pointXYZMap);

    mmind::api::PointXYZBGRMap pointXYZBGRMap;
    device.capturePointXYZBGRMap(pointXYZBGRMap);

    std::unique_lock<std::mutex> lock(m);

    cv::imwrite(colorFile, color8UC3);
    std::cout << "Capture and save color image : " << colorFile << std::endl;

    cv::imwrite(depthFile, depth8U);
    std::cout << "Capture and save depth image : " << depthFile << std::endl;

    std::string pointCloudPath = "pointCloud_" + id + ".ply";
    savePLY(pointXYZMap, pointCloudPath);

    std::string pointCloudColorPath = "colorPointCloud_" + id + ".ply";
    savePLY(pointXYZBGRMap, pointCloudColorPath);
}
