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
    laserSettings.FringeCodingMode = mmind::api::LaserSettings::LaserFringeCodingMode::Fast;
    showError(device.setLaserSettings(laserSettings));

    device.disconnect();
    std::cout << "Disconnect Mech-Eye Success." << std::endl;
    return 0;
}
