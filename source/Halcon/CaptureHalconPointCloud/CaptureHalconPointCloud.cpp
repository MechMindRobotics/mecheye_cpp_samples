/*******************************************************************************
 *BSD 3-Clause License
 *
 *Copyright (c) 2016-2023, Mech-Mind Robotics
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

/*
With this sample program, you can obtain point cloud data from a camera, and then transform and save
the point clouds using HALCON C++ interface.
*/

#include <iostream>
#include <thread>
#include <algorithm>
#include <chrono>
#include <iostream>

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include "halconcpp/HalconCpp.h"
#include "HalconUtil.h"

int main()
{
    // Open and select camera
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    // Get white point cloud and convert the format to Halcon
    mmind::api::PointXYZMap pointXYZMap;
    showError(device.capturePointXYZMap(pointXYZMap));

    const auto halconPointCloudXYZ = mecheyeToHalconPointCloud(pointXYZMap);

    // Save the white point cloud
    const auto pointCloudFileXYZ = "MechEyePointXYZMap.ply";
    std::cout << "Saving point cloud to file: " << pointCloudFileXYZ << std::endl;
    savePointCloud(halconPointCloudXYZ, pointCloudFileXYZ);

    // Get color point cloud and convert the format to Halcon
    mmind::api::PointXYZBGRMap pointXYZBGRMap;
    showError(device.capturePointXYZBGRMap(pointXYZBGRMap));

    const auto halconPointCloudXYZRGB = mecheyeToHalconPointCloud(pointXYZBGRMap);

    // Save the color point cloud
    const auto pointCloudFileXYZRGB = "MechEyePointXYZBGRMap.ply";
    std::cout << "Saving point cloud to file: " << pointCloudFileXYZRGB << std::endl;
    savePointCloud(halconPointCloudXYZRGB, pointCloudFileXYZRGB);

    // Disconnect the camera and exit
    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
