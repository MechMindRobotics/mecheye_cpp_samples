﻿/*******************************************************************************
 *BSD 3-Clause License
 *
 *Copyright (c) 2016-2024, Mech-Mind Robotics
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
With this sample, you can obtain and save 2D images, depth maps and point clouds
sequentially from multiple cameras.
*/

#include <opencv2/imgcodecs.hpp>
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/api_util.h"

int main()
{
    std::vector<mmind::eye::Camera> cameraList = findAndConnectMultiCamera();

    if (cameraList.empty()) {
        std::cout << "No cameras connected." << std::endl;
        return 0;
    }

    if (!confirmCapture3D()) {
        for (auto& camera : cameraList)
            camera.disconnect();
        return 0;
    }

    // Start capturing images
    for (auto& camera : cameraList) {
        mmind::eye::CameraInfo cameraInfo;
        showError(camera.getCameraInfo(cameraInfo));

        mmind::eye::Frame2DAnd3D frame2DAnd3D;
        showError(camera.capture2DAnd3D(frame2DAnd3D));

        const std::string colorFile = cameraInfo.serialNumber.empty()
                                          ? "2DImage.png"
                                          : "2DImage_" + cameraInfo.serialNumber + ".png";
        mmind::eye::Color2DImage colorImage = frame2DAnd3D.frame2D().getColorImage();
        cv::Mat color8UC3 =
            cv::Mat(colorImage.height(), colorImage.width(), CV_8UC3, colorImage.data());
        cv::imwrite(colorFile, color8UC3);
        std::cout << "Capture and save the 2D image: " << colorFile << std::endl;

        const std::string depthFile = cameraInfo.serialNumber.empty()
                                          ? "DepthMap.tiff"
                                          : "DepthMap_" + cameraInfo.serialNumber + ".tiff";
        mmind::eye::DepthMap depthMap = frame2DAnd3D.frame3D().getDepthMap();
        cv::Mat depth32F = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());
        cv::imwrite(depthFile, depth32F);
        std::cout << "Capture and save the depth map: " << depthFile << std::endl;

        const std::string pointCloudFile = cameraInfo.serialNumber.empty()
                                               ? "PointCloud.ply"
                                               : "PointCloud_" + cameraInfo.serialNumber + ".ply";
        showError(frame2DAnd3D.frame3D().saveUntexturedPointCloud(mmind::eye::FileFormat::PLY,
                                                                  pointCloudFile));
        std::cout << "Capture and save the untextured point cloud: " << pointCloudFile << std::endl;

        const std::string texturedPointCloudFile =
            cameraInfo.serialNumber.empty()
                ? "TexturedPointCloud.ply"
                : "TexturedPointCloud_" + cameraInfo.serialNumber + ".ply";
        showError(frame2DAnd3D.saveTexturedPointCloud(mmind::eye::FileFormat::PLY,
                                                      texturedPointCloudFile));
        std::cout << "Capture and save the textured point cloud: " << texturedPointCloudFile
                  << std::endl;

        camera.disconnect();
    }

    return 0;
}
