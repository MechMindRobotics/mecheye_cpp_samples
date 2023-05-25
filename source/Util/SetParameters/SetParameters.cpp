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
With this sample program, you can set specified parameters to a camera.
*/

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include <iostream>

int main()
{
    // List all available cameras and connect to a camera by the displayed index.
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    // Obtain the basic information of the connected camera.
    mmind::api::MechEyeDeviceInfo deviceInfo;
    showError(device.getDeviceInfo(deviceInfo));
    printDeviceInfo(deviceInfo);

    // Obtain the names of all parameter groups.
    std::vector<std::string> userSets;
    showError(device.getAllUserSets(userSets));

    std::cout << "All user sets : ";
    for (size_t i = 0; i < userSets.size(); i++)
        std::cout << userSets[i] << "  ";
    std::cout << std::endl;

    // Obtain the name of the currently selected parameter group.
    std::string currentUserSet;
    showError(device.getCurrentUserSet(currentUserSet));
    std::cout << "Current user set : " << currentUserSet << std::endl;

    // Select a parameter group by its name.
    showError(device.setCurrentUserSet(userSets.front()));
    std::cout << "Set \"" << userSets.front() << "\" as the current user set." << std::endl
              << std::endl;

    // Set the exposure times for acquiring depth information.
    showError(device.setScan3DExposure(std::vector<double>{5, 10}));

    // Obtain the current exposure times for acquiring depth information for checking.
    std::vector<double> exposureSequence;
    showError(device.getScan3DExposure(exposureSequence));

    std::cout << "The 3D scanning exposure multiplier : " << exposureSequence.size() << "."
              << std::endl;
    for (size_t i = 0; i < exposureSequence.size(); i++)
        std::cout << "3D scanning exposure time " << i + 1 << " : " << exposureSequence[i] << " ms."
                  << std::endl;

    // Set the range of depth values to retain, and then obtain the parameter values for checking.
    showError(device.setDepthRange(mmind::api::DepthRange(100, 1000)));
    mmind::api::DepthRange depthRange;
    showError(device.getDepthRange(depthRange));
    std::cout << "3D scanning depth Lower Limit : " << depthRange.lower
              << " mm, depth upper limit : " << depthRange.upper << " mm." << std::endl;

    // Set the ROI for the depth map and point cloud, and then obtain the parameter values for checking.
    showError(device.setScan3DROI(mmind::api::ROI(0, 0, 500, 500)));
    mmind::api::ROI scan3dRoi;
    showError(device.getScan3DROI(scan3dRoi));
    std::cout << "3D scanning ROI topLeftX : " << scan3dRoi.x << ", topLeftY : " << scan3dRoi.y
              << ", width : " << scan3dRoi.width << ", height : " << scan3dRoi.height << std::endl;

    // Set the exposure mode and exposure time for capturing the 2D image, and then obtain the parameter values for checking.
    showError(
        device.setScan2DExposureMode(mmind::api::Scanning2DSettings::Scan2DExposureMode::Timed));
    showError(device.setScan2DExposureTime(100));

    mmind::api::Scanning2DSettings::Scan2DExposureMode exposureMode2D;
    double scan2DExposureTime;
    showError(device.getScan2DExposureMode(exposureMode2D));
    showError(device.getScan2DExposureTime(scan2DExposureTime));
    std::cout << "2D scanning exposure mode enum : " << static_cast<int>(exposureMode2D)
              << ", exposure time : " << scan2DExposureTime << " ms." << std::endl;

    // Set the point cloud processing parameters, and obtain the parameter values for checking.
    showError(device.setCloudSurfaceSmoothingMode(
        mmind::api::PointCloudProcessingSettings::PointCloudSurfaceSmoothing::Normal));
    showError(device.setCloudNoiseRemovalMode(
        mmind::api::PointCloudProcessingSettings::PointCloudNoiseRemoval::Normal));
    showError(device.setCloudOutlierRemovalMode(
        mmind::api::PointCloudProcessingSettings::PointCloudOutlierRemoval::Normal));
    showError(device.setCloudEdgePreservationMode(
        mmind::api::PointCloudProcessingSettings::PointCloudEdgePreservation::Normal));

    mmind::api::PointCloudProcessingSettings::PointCloudSurfaceSmoothing surfaceSmoothing;
    mmind::api::PointCloudProcessingSettings::PointCloudNoiseRemoval noiseRemoval;
    mmind::api::PointCloudProcessingSettings::PointCloudOutlierRemoval outlierRemoval;
    mmind::api::PointCloudProcessingSettings::PointCloudEdgePreservation edgePreservation;

    showError(device.getCloudSurfaceSmoothingMode(surfaceSmoothing));
    showError(device.getCloudNoiseRemovalMode(noiseRemoval));
    showError(device.getCloudOutlierRemovalMode(outlierRemoval));
    showError(device.getCloudEdgePreservationMode(edgePreservation));

    std::cout << "Cloud surface smoothing mode enum : " << static_cast<int>(surfaceSmoothing)
              << std::endl;
    std::cout << "Cloud noise removal mode enum : " << static_cast<int>(noiseRemoval) << std::endl;
    std::cout << "Cloud outlier removal mode enum : " << static_cast<int>(outlierRemoval)
              << std::endl;
    std::cout << "Cloud edge preservation mode enum : " << static_cast<int>(edgePreservation)
              << std::endl;

    // Save all the parameter settings to the currently selected parameter group.
    showError(device.saveAllSettingsToUserSets());
    std::cout << "Save all parameters to current user set." << std::endl;

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
