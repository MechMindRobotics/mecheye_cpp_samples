#include <iostream>
#include "MechEyeApi.h"
#include "SampleUtil.h"
#include "OpenCVUtil.h"

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    mmind::api::ColorMap color;
    showError(device.captureColorMap(color));
    const std::string colorFile = "ColorMap.png";
    saveMap(color, colorFile);

    device.disconnect();
    std::cout << "Disconnect Mech-Eye Success." << std::endl;
    return 0;
}
