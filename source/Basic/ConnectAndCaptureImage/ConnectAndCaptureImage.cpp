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

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include <iostream>

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    mmind::api::DeviceResolution deviceResolution;
    showError(device.getDeviceResolution(deviceResolution));
    printDeviceResolution(deviceResolution);

    mmind::api::ColorMap colorMap;
    unsigned row = 0;
    unsigned col = 0;
    showError(device.captureColorMap(colorMap));
    std::cout << "Color map size is width: " << colorMap.width() << " height: " << colorMap.height()
              << "." << std::endl;
    try {
        mmind::api::ElementColor colorElem = colorMap.at(row, col);
        std::cout << "Color map element at ( " << row << "," << col
                  << ") is R: " << unsigned(colorElem.r) << " G: " << unsigned(colorElem.g)
                  << " B: " << unsigned(colorElem.b) << "." << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        device.disconnect();
        return 0;
    }

    mmind::api::DepthMap depthMap;
    showError(device.captureDepthMap(depthMap));
    std::cout << "Depth map size is width: " << depthMap.width() << " height: " << depthMap.height()
              << "." << std::endl;
    try {
        mmind::api::ElementDepth depthElem = depthMap.at(row, col);
        std::cout << "Depth map element at ( " << row << "," << col << ") is depth: " << depthElem.d
                  << " mm." << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        device.disconnect();
        return 0;
    }

    mmind::api::PointXYZMap pointXYZMap;
    showError(device.capturePointXYZMap(pointXYZMap));
    std::cout << "Pointcloud map size is width: " << pointXYZMap.width()
              << " height: " << pointXYZMap.height() << "." << std::endl;
    try {
        mmind::api::ElementPointXYZ pointXYZElem = pointXYZMap.at(row, col);
        std::cout << "PointXYZ map element at ( " << row << "," << col
                  << ") is X: " << pointXYZElem.x << " mm  Y: " << pointXYZElem.y
                  << " mm  Z:" << pointXYZElem.z << " mm." << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        device.disconnect();
        return 0;
    }

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
