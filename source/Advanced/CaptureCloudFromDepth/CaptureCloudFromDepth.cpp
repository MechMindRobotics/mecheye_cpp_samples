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

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "MechEyeApi.h"
#include "SampleUtil.h"

void savePLY(const mmind::api::DepthMap& depth, const std::string& path,
             const mmind::api::DeviceIntri& intri);
void savePLY(const mmind::api::DepthMap& depth, const mmind::api::ColorMap& color,
             const std::string& path, const mmind::api::DeviceIntri& intri);

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    mmind::api::MechEyeDeviceInfo deviceInfo;
    showError(device.getDeviceInfo(deviceInfo));
    printDeviceInfo(deviceInfo);

    mmind::api::ColorMap color;
    showError(device.captureColorMap(color));

    mmind::api::DepthMap depth;
    showError(device.captureDepthMap(depth));

    mmind::api::DeviceIntri deviceIntri;
    showError(device.getDeviceIntri(deviceIntri));

    std::string pointCloudPath = "PointCloudXYZ.ply";
    savePLY(depth, pointCloudPath, deviceIntri);

    std::string pointCloudColorPath = "PointCloudXYZRGB.ply";
    savePLY(depth, color, pointCloudColorPath, deviceIntri);

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}

void savePLY(const mmind::api::DepthMap& depth, const std::string& path,
             const mmind::api::DeviceIntri& intri)
{
    // write pointcloudXYZ data
    pcl::PointCloud<pcl::PointXYZ> pointCloud(depth.width(), depth.height());
    uint32_t size = depth.height() * depth.width();
    pointCloud.resize(size);

    for (int m = 0; m < depth.height(); ++m)
        for (int n = 0; n < depth.width(); ++n) {
            float d;
            try {
                d = depth.at(m, n).d;
            } catch (const std::exception& e) {
                std::cout << "Exception: " << e.what() << std::endl;
                return;
            }
            pointCloud.at(n, m).z = d;
            pointCloud.at(n, m).x = (n - intri.cameraMatrix[2]) * d / intri.cameraMatrix[0];
            pointCloud.at(n, m).y = (m - intri.cameraMatrix[3]) * d / intri.cameraMatrix[1];
        }

    pcl::PLYWriter writer;
    writer.write(path, pointCloud, true);
    std::cout << "PointCloudXYZ has : " << pointCloud.width * pointCloud.height << " data points."
              << std::endl;
    std::cout << "PointCloudXYZ saved to: " << path << std::endl;

    return;
}

void savePLY(const mmind::api::DepthMap& depth, const mmind::api::ColorMap& color,
             const std::string& path, const mmind::api::DeviceIntri& intri)
{
    // write pointcloudXYZRGB data
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud(depth.width(), depth.height());
    uint32_t size = depth.height() * depth.width();
    pointCloud.resize(size);

    for (int m = 0; m < depth.height(); ++m)
        for (int n = 0; n < depth.width(); ++n) {
            float d;
            uint8_t r, g, b;
            try {
                d = depth.at(m, n).d;
                r = color.at(m, n).r;
                g = color.at(m, n).g;
                b = color.at(m, n).b;
            } catch (const std::exception& e) {
                std::cout << "Exception: " << e.what() << std::endl;
                return;
            }
            pointCloud.at(n, m).z = d;
            pointCloud.at(n, m).x = (n - intri.cameraMatrix[2]) * d / intri.cameraMatrix[0];
            pointCloud.at(n, m).y = (m - intri.cameraMatrix[3]) * d / intri.cameraMatrix[1];

            pointCloud.at(n, m).r = r;
            pointCloud.at(n, m).g = g;
            pointCloud.at(n, m).b = b;
        }

    pcl::PLYWriter writer;
    writer.write(path, pointCloud, true);
    std::cout << "PointCloudXYZRGB has : " << pointCloud.width * pointCloud.height
              << " data points." << std::endl;
    std::cout << "PointCloudXYZRGB saved to: " << path << std::endl;

    return;
}
