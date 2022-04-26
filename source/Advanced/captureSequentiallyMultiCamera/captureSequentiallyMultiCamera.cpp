#include <iostream>

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include "CaptureUtil.h"

int main()
{
    std::pair<mmind::api::MechEyeDevice*, int> pair = findAndConnectMulti();
    mmind::api::MechEyeDevice* devices = pair.first;
    int size = pair.second;

    for (int i = 0; i < size; ++i) {
        mmind::api::MechEyeDeviceInfo info;
        showError(devices[i].getDeviceInfo(info));
        std::string id = info.id;

        capture(devices[i], id);

        devices[i].disconnect();
    }

    delete[] devices;
    return 0;
}
