/*******************************************************************************
 *BSD 3-Clause License
 *
 *Copyright (c) 2016-2022, Mech-Mind Robotics
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 *1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 *3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

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
