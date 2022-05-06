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
    laserSettings.PowerLevel = 40;
    showError(device.setLaserSettings(laserSettings));

    showError(device.getLaserSettings(laserSettings));
    std::cout << "PowerLevel:" << laserSettings.PowerLevel << std::endl;

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
