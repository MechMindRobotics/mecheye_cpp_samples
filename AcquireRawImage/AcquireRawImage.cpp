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
This is a simple example of how to find and connect an available Mech-Eye Device
and then capture a line scan image and convert to OpenCV format.
*/

#include <iostream>
#include <opencv2/opencv.hpp>
#include "profiler/parameters/RawImageParameters.h"
#include "profiler/Profiler.h"
#include "profiler/api_util.h"

int main()
{
    mmind::eye::Profiler profiler;
    if (!findAndConnect(profiler))
        return -1;

    // Set Exposure Time
    mmind::eye::UserSet currentUserSet = profiler.currentUserSet();
    showError(currentUserSet.setIntValue(mmind::eye::brightness_settings::ExposureTime::name, 60));

    // set ZNarrowing
    showError(currentUserSet.setEnumValue(
        mmind::eye::roi::ZDirectionRoi::name,
        static_cast<int>(mmind::eye::roi::ZDirectionRoi::Value::ImageHeight_1_1)));

    mmind::eye::RawImage rawImage;
    showError(profiler.captureRawImage(rawImage));
    const std::string imageFile = "LineScanImage.png";
    cv::Mat image =
        cv::Mat(rawImage.height(), rawImage.width(), CV_8UC1, rawImage.getData().data());
    cv::imshow(imageFile, image);
    cv::imwrite(imageFile, image);
    std::cout << "Capture and save LineScan raw image : " << imageFile << std::endl;
    cv::waitKey(0);

    profiler.disconnect();
    std::cout << "Disconnected from the Mech-Eye profiler successfully." << std::endl;
    return 0;
}
