#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <set>
#include <vector>

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include "OpenCVUtil.h"
#include "PclUtil.h"

void captureAsync(mmind::api::MechEyeDevice& device, std::mutex& m);

int main()
{
    std::pair<mmind::api::MechEyeDevice*, int> pair = findAndConnectMulti();
    mmind::api::MechEyeDevice* devices = pair.first;
    int size = pair.second;
    if (!devices || size == 0)
        return -1;

    std::vector<std::future<void>> container;
    std::mutex m;
    for (int i = 0; i < size; ++i) {
        container.emplace_back(
            std::async(std::launch::async, captureAsync, std::ref(devices[i]), std::ref(m)));
    }

    for (int i = 0; i < size; ++i) {
        container[i].get();
        devices[i].disconnect();
    }

    delete[] devices;
    return 0;
}

void captureAsync(mmind::api::MechEyeDevice& device, std::mutex& m)
{
    mmind::api::MechEyeDeviceInfo info;
    showError(device.getDeviceInfo(info));
    std::string id = info.id;

    mmind::api::ColorMap color;
    showError(device.captureColorMap(color));

    mmind::api::DepthMap depth;
    showError(device.captureDepthMap(depth));

    mmind::api::PointXYZMap pointXYZMap;
    device.capturePointXYZMap(pointXYZMap);

    mmind::api::PointXYZBGRMap pointXYZBGRMap;
    device.capturePointXYZBGRMap(pointXYZBGRMap);

    std::unique_lock<std::mutex> lock(m);

    const std::string colorFile = "ColorMap_" + id + ".png";
    saveMap(color, colorFile);

    const std::string depthFile = "DepthMap_" + id + ".png";
    saveMap(depth, depthFile);

    std::string pointCloudPath = "PointCloud_" + id + ".ply";
    savePLY(pointXYZMap, pointCloudPath);

    std::string pointCloudColorPath = "ColorPointCloud_" + id + ".ply";
    savePLY(pointXYZBGRMap, pointCloudColorPath);
}
