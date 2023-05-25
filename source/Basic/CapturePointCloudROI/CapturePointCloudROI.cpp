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
With this sample program, you can obtain and save untextured and textured point clouds (PCL format)
of the objects in the ROI from a camera.
*/

#include <iostream>
#include <thread>

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include "PclUtil.h"

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    // Set 3D ROI, MechEye UHP serials in capture mode 'Merge' does not support this parameter.
    showError(device.setScan3DROI(mmind::api::ROI(0, 0, 500, 500)));

    mmind::api::PointXYZMap pointXYZMap;
    showError(device.capturePointXYZMap(pointXYZMap));

    mmind::api::PointXYZBGRMap pointXYZBGRMap;
    showError(device.capturePointXYZBGRMap(pointXYZBGRMap));

    std::string pointCloudPath = "PointCloudXYZ.ply";
    pcl::PointCloud<pcl::PointXYZ> pointCloud(pointXYZMap.width(), pointXYZMap.height());
    toPCL(pointCloud, pointXYZMap);
    viewPCL(pointCloud);
    savePLY(pointXYZMap, pointCloudPath);

    std::string colorPointCloudPath = "PointCloudXYZRGB.ply";
    pcl::PointCloud<pcl::PointXYZRGB> colorPointCloud(pointXYZBGRMap.width(),
                                                      pointXYZBGRMap.height());
    toPCL(colorPointCloud, pointXYZBGRMap);
    viewPCL(colorPointCloud);
    savePLY(pointXYZBGRMap, colorPointCloudPath);

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;

    return 0;
}
