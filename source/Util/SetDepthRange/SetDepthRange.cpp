#include "MechEyeApi.h"
#include "SampleUtil.h"
#include <iostream>

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    showError(device.setDepthRange(mmind::api::DepthRange(100, 1000)));
    mmind::api::DepthRange depthRange;
    showError(device.getDepthRange(depthRange));
    std::cout << "3D scanning depth Lower Limit : " << depthRange.lower
              << " mm, depth upper limit : " << depthRange.upper << " mm." << std::endl;

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
