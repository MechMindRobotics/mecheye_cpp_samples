#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "MechEyeApi.h"
#include "SampleUtil.h"

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    mmind::api::ColorMap color;
    showError(device.captureColorMap(color));
    const std::string colorFile = "colorMap.png";
    cv::Mat color8UC3 = cv::Mat(color.height(), color.width(), CV_8UC3, color.data());
    cv::imwrite(colorFile, color8UC3);
    std::cout << "Capture and save color image : " << colorFile << std::endl;

    device.disconnect();
    std::cout << "Disconnect Mech-Eye Success." << std::endl;
    return 0;
}
