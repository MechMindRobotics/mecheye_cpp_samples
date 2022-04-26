#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "MechEyeApi.h"
#include "SampleUtil.h"

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

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
