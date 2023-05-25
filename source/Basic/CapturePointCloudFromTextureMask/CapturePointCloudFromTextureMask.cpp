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
With this sample program, you can construct and save untextured and textured point clouds (PCL
format) generated from a depth map and masked 2D image.
*/

#include <iostream>

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include "PclUtil.h"

bool contains(const mmind::api::ROI& roi, int x, int y)
{
    return x >= roi.x && x < roi.x + roi.width && y >= roi.y && y < roi.y + roi.height;
}

mmind::api::ColorMap generateTextureMask(const mmind::api::ColorMap& color,
                                         const mmind::api::ROI& roi1, const mmind::api::ROI& roi2)
{
    mmind::api::ColorMap colorMask;
    const int width = color.width();
    const int height = color.height();
    colorMask.resize(width, height);
    for (int r = 0; r < height; ++r) {
        for (int c = 0; c < width; ++c) {
            if (contains(roi1, c, r) || contains(roi2, c, r)) {
                colorMask.at(r, c).b = 1;
                colorMask.at(r, c).g = 1;
                colorMask.at(r, c).r = 1;
            }
        }
    }
    return colorMask;
}

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    mmind::api::ColorMap color;
    showError(device.captureColorMap(color));
    mmind::api::DepthMap depth;
    showError(device.captureDepthMap(depth));
    mmind::api::DeviceIntri deviceIntri;
    device.getDeviceIntri(deviceIntri);
    printCalibParams(deviceIntri);

    const int width = color.width();
    const int height = color.height();
    mmind::api::ROI roi1(width / 5, height / 5, width / 2, height / 2);
    mmind::api::ROI roi2(width * 2 / 5, height * 2 / 5, width / 2, height / 2);

    /**
     *  Generate a mask in shape of
     *   ______________________________
     *  |                              |
     *  |                              |
     *  |   *****************          |
     *  |   *****************          |
     *  |   ************************   |
     *  |   ************************   |
     *  |          *****************   |
     *  |          *****************   |
     *  |                              |
     *  |______________________________|
     */
    mmind::api::ColorMap colorMask = generateTextureMask(color, roi1, roi2);

    mmind::api::PointXYZMap pointXYZROIMap;
    showError(mmind::api::MechEyeDevice::getCloudFromTextureMask(depth, colorMask, deviceIntri,
                                                                 pointXYZROIMap));
    std::string pointCloudPath = "PointCloudXYZROI.ply";
    pcl::PointCloud<pcl::PointXYZ> pointCloudROI(pointXYZROIMap.width(), pointXYZROIMap.height());
    toPCL(pointCloudROI, pointXYZROIMap);
    viewPCL(pointCloudROI);
    savePLY(pointXYZROIMap, pointCloudPath);

    mmind::api::PointXYZBGRMap pointXYZBGRROIMap;
    showError(mmind::api::MechEyeDevice::getCloudFromTextureMask(depth, colorMask, color,
                                                                 deviceIntri, pointXYZBGRROIMap));
    std::string colorPointCloudPath = "PointCloudXYZRGBROI.ply";
    pcl::PointCloud<pcl::PointXYZRGB> colorPointCloud(pointXYZBGRROIMap.width(),
                                                      pointXYZBGRROIMap.height());
    toPCL(colorPointCloud, pointXYZBGRROIMap);
    viewPCL(colorPointCloud);
    savePLY(pointXYZBGRROIMap, colorPointCloudPath);

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;

    return 0;
}
