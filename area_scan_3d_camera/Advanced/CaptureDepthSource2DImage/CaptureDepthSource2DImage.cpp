/*******************************************************************************
 *BSD 3-Clause License
 *
 *Copyright (c) 2016-2025, Mech-Mind Robotics Technologies Co., Ltd.
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
With this sample, you can obtain and save the 2D image from the depth source camera.
*/

#include <opencv2/opencv.hpp>
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/api_util.h"

int main()
{
    mmind::eye::Camera camera;
    if (!findAndConnect(camera))
        return -1;

    mmind::eye::Frame2D frame2D;
    auto errorStatus = camera.captureDepthSource2D(frame2D);
    if (!errorStatus.isOK()) {
        showError(errorStatus);
        return -1;
    }

    cv::Mat image2D;
    const std::string imageFile = "DepthSource2DImage.png";

    switch (frame2D.colorType()) {
    case mmind::eye::ColorTypeOf2DCamera::Monochrome:
    {
        mmind::eye::GrayScale2DImage grayImage = frame2D.getGrayScaleImage();
        image2D = cv::Mat(grayImage.height(), grayImage.width(), CV_8UC1, grayImage.data());
        break;
    }
    case mmind::eye::ColorTypeOf2DCamera::Color:
    {
        mmind::eye::Color2DImage colorImage = frame2D.getColorImage();
        image2D = cv::Mat(colorImage.height(), colorImage.width(), CV_8UC3, colorImage.data());
        break;
    }
    default:
        std::cerr << "The depth source 2D image has an unsupported pixel type." << std::endl;
        camera.disconnect();
        return -1;
    }

    cv::imwrite(imageFile, image2D);
    std::cout << "Capture and save the 2D image from the depth source camera: " << imageFile
              << std::endl;

    camera.disconnect();
    std::cout << "Disconnected from the camera successfully." << std::endl;
    return 0;
}