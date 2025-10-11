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
With this sample, you can obtain and save the depth map rendered with the jet color scheme.
*/

#include <opencv2/opencv.hpp>
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/api_util.h"

namespace {

inline bool isApprox0(double d) { return std::fabs(d) <= DBL_EPSILON; }

cv::Mat renderDepthData(const cv::Mat& depth)
{
    if (depth.empty())
        return cv::Mat();
    cv::Mat mask = cv::Mat(depth == depth);
    double minDepthValue, maxDepthValue;
    cv::minMaxLoc(depth, &minDepthValue, &maxDepthValue, nullptr, nullptr, mask);

    cv::Mat depth8U;
    isApprox0(maxDepthValue - minDepthValue)
        ? depth.convertTo(depth8U, CV_8UC1)
        : depth.convertTo(depth8U, CV_8UC1, (255.0 / (minDepthValue - maxDepthValue)),
                          (((maxDepthValue * 255.0) / (maxDepthValue - minDepthValue)) + 1));

    if (depth8U.empty())
        return cv::Mat();

    cv::Mat coloredDepth;
    cv::applyColorMap(depth8U, coloredDepth, cv::COLORMAP_JET);
    coloredDepth.forEach<cv::Vec3b>([&](auto& val, const int* pos) {
        if (!depth8U.ptr<uchar>(pos[0])[pos[1]]) {
            val[0] = 0;
            val[1] = 0;
            val[2] = 0;
        }
    });
    return coloredDepth;
}

} // namespace

int main()
{
    mmind::eye::Camera camera;
    if (!findAndConnect(camera))
        return -1;

    if (!confirmCapture3D()) {
        camera.disconnect();
        return 0;
    }
    mmind::eye::Frame3D frame3D;
    showError(camera.capture3D(frame3D));

    mmind::eye::DepthMap depthMap = frame3D.getDepthMap();
    cv::Mat depth32F = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());
    cv::Mat renderedDepth = renderDepthData(depth32F);
    const std::string renderedDepthImgFile = "RenderedDepthMap.tiff";
    cv::imwrite(renderedDepthImgFile, renderedDepth);
    std::cout << "Capture and save the depth map: " << renderedDepthImgFile << std::endl;

    camera.disconnect();
    std::cout << "Disconnected from the camera successfully." << std::endl;
    return 0;
}
