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

    mmind::api::MechEyeDeviceInfo deviceInfo;
    showError(device.getDeviceInfo(deviceInfo));
    printDeviceInfo(deviceInfo);

    std::vector<std::string> userSets;
    showError(device.getAllUserSets(userSets));

    std::cout << "All user sets : ";
    for (size_t i = 0; i < userSets.size(); i++)
        std::cout << userSets[i] << "  ";
    std::cout << std::endl;

    std::string currentUserSet;
    showError(device.getCurrentUserSet(currentUserSet));
    std::cout << "Current user set : " << currentUserSet << std::endl;

    showError(device.setCurrentUserSet(userSets.front()));
    std::cout << "Set \"" << userSets.front() << "\" as the current user set." << std::endl
              << std::endl;

    showError(device.setScan3DExposure(std::vector<double>{5, 10}));

    std::vector<double> exposureSequence;
    showError(device.getScan3DExposure(exposureSequence));

    std::cout << "The 3D scanning exposure multiplier : " << exposureSequence.size() << "."
              << std::endl;
    for (size_t i = 0; i < exposureSequence.size(); i++)
        std::cout << "3D scanning exposure time " << i + 1 << " : " << exposureSequence[i] << " ms."
                  << std::endl;

    showError(device.setDepthRange(mmind::api::DepthRange(100, 1000)));
    mmind::api::DepthRange depthRange;
    showError(device.getDepthRange(depthRange));
    std::cout << "3D scanning depth Lower Limit : " << depthRange.lower
              << " mm, depth upper limit : " << depthRange.upper << " mm." << std::endl;

    showError(device.setScan3DROI(mmind::api::ROI(0, 0, 500, 500)));
    mmind::api::ROI scan3dRoi;
    showError(device.getScan3DROI(scan3dRoi));
    std::cout << "3D scanning ROI topLeftX : " << scan3dRoi.x << ", topLeftY : " << scan3dRoi.y
              << ", width : " << scan3dRoi.width << ", height : " << scan3dRoi.height << std::endl;

    showError(
        device.setScan2DExposureMode(mmind::api::Scanning2DSettings::Scan2DExposureMode::Timed));
    showError(device.setScan2DExposureTime(100));

    mmind::api::Scanning2DSettings::Scan2DExposureMode exposureMode2D;
    double scan2DExposureTime;
    showError(device.getScan2DExposureMode(exposureMode2D));
    showError(device.getScan2DExposureTime(scan2DExposureTime));
    std::cout << "2D scanning exposure mode enum : " << static_cast<int>(exposureMode2D)
              << ", exposure time : " << scan2DExposureTime << " ms." << std::endl;

    showError(device.setCloudSmoothMode(
        mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Normal));
    showError(device.setCloudOutlierFilterMode(
        mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Normal));

    mmind::api::PointCloudProcessingSettings::CloudSmoothMode cloudSmoothMode;
    mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode cloudOutlierFilterMode;
    showError(device.getCloudSmoothMode(cloudSmoothMode));
    showError(device.getCloudOutlierFilterMode(cloudOutlierFilterMode));

    std::cout << "Cloud smooth mode enum : " << static_cast<int>(cloudSmoothMode)
              << ", cloud outlier filter mode enum : " << static_cast<int>(cloudOutlierFilterMode)
              << std::endl;

    showError(device.saveAllSettingsToUserSets());
    std::cout << "Save all parameters to current user set." << std::endl;

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
