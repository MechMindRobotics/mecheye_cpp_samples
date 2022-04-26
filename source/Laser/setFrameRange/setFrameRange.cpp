#include <iostream>

#include "MechEyeApi.h"
#include "SampleUtil.h"

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;
   
    mmind::api::LaserSettings laserSettings;
    showError(device.getLaserSettings(laserSettings));
    laserSettings.FrameRangeStart = 51;
    laserSettings.FrameRangeEnd = 90;
    showError(device.setLaserSettings(laserSettings));

    showError(device.getLaserSettings(laserSettings));
    std::cout << "FrameRangeStart:" << laserSettings.FrameRangeStart << ","
              << "FrameRangeEnd:" << laserSettings.FrameRangeEnd << std::endl;

    device.disconnect();
    std::cout << "Disconnect Mech-Eye Success." << std::endl;
    return 0;
}
