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
    std::cout << "Old FramePartitionCount:" << laserSettings.FramePartitionCount
              << std::endl;
    laserSettings.FramePartitionCount = 2;
    showError(device.setLaserSettings(laserSettings));

    showError(device.getLaserSettings(laserSettings));
    std::cout << "New FramePartitionCount:"
         << laserSettings.FramePartitionCount << std::endl;

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
