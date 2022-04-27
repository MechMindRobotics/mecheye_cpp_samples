#include "MechEyeApi.h"
#include "SampleUtil.h"
#include <iostream>

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    mmind::api::DeviceIntri deviceIntri;
    showError(device.getDeviceIntri(deviceIntri));
    printCalibParams(deviceIntri);

    device.disconnect();
    std::cout << "Disconnect Mech-Eye Success." << std::endl;
    return 0;
}
