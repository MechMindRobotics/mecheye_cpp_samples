#include <chrono>
#include <iostream>
#include <thread>

#include <opencv2/highgui/highgui.hpp>

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include "PclUtil.h"

int main()
{
    const auto captureTime = std::chrono::minutes(5);
    const auto capturePeriod = std::chrono::seconds(10);

    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    mmind::api::MechEyeDeviceInfo deviceInfo;
    showError(device.getDeviceInfo(deviceInfo));
    printDeviceInfo(deviceInfo);

    std::cout << "Starting capturing for " << captureTime.count() << " minutes." << std::endl;

    const auto start = std::chrono::high_resolution_clock::now();

    while (std::chrono::high_resolution_clock::now() - start < captureTime) {
        const auto before = std::chrono::high_resolution_clock::now();

        std::ostringstream ss;
        ss << (std::chrono::duration_cast<std::chrono::seconds>(before - start)).count();
        std::string time = ss.str();

        mmind::api::ColorMap color;
        showError(device.captureColorMap(color));
        const std::string colorFile = "colorMap_" + time + ".png";
        cv::Mat color8UC3 = cv::Mat(color.height(), color.width(), CV_8UC3, color.data());
        cv::imwrite(colorFile, color8UC3);
        std::cout << "Capture and save color image : " << colorFile << std::endl;

        mmind::api::DepthMap depth;
        showError(device.captureDepthMap(depth));
        const std::string depthFile = "depthMap_" + time + ".png";
        cv::Mat depth8U;
        cv::Mat depth32F = cv::Mat(depth.height(), depth.width(), CV_32FC1, depth.data());
        double minDepth, maxDepth;
        cv::minMaxLoc(depth32F, &minDepth, &maxDepth);
        depth32F.convertTo(depth8U, CV_8UC1, 255.0 / (maxDepth));
        cv::imwrite(depthFile, depth8U);
        std::cout << "Capture and save depth image : " << depthFile << std::endl;

        mmind::api::PointXYZMap pointXYZMap;
        device.capturePointXYZMap(pointXYZMap);

        mmind::api::PointXYZBGRMap pointXYZBGRMap;
        device.capturePointXYZBGRMap(pointXYZBGRMap);

        std::string pointCloudPath = "pointCloud_" + time + ".ply";
        savePLY(pointXYZMap, pointCloudPath);

        std::string pointCloudColorPath = "colorPointCloud_" + time + ".ply";
        savePLY(pointXYZBGRMap, pointCloudColorPath);

        const auto after = std::chrono::high_resolution_clock::now();
        const auto timeUsed = after - before;
        if (timeUsed < capturePeriod)
            std::this_thread::sleep_for(capturePeriod - timeUsed);
        else
            std::cout << "Your capture time is longer than your capture period. Please increase "
                         "your capture period."
                      << std::endl;

        const auto timeRemaining =
            captureTime - (std::chrono::high_resolution_clock::now() - start);
        const auto remainingMinutes =
            std::chrono::duration_cast<std::chrono::minutes>(timeRemaining);
        const auto remainingSeconds =
            std::chrono::duration_cast<std::chrono::seconds>(timeRemaining - remainingMinutes);
        std::cout << "Remaining time: " << remainingMinutes.count() << " minutes and "
                  << remainingSeconds.count() << "seconds." << std::endl;
    }

    std::cout << "Capturing completed for " << captureTime.count() << " minutes." << std::endl;

    device.disconnect();
    std::cout << "Disconnect Mech-Eye Success." << std::endl;
    return 0;
}
