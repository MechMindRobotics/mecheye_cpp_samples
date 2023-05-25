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
With this sample program, you can set the capture mode (capture images with 2D camera 1, with 2D
camera 2, or with both 2D cameras and compose the outputs).
*/

#include <iostream>

#include "MechEyeApi.h"
#include "SampleUtil.h"
namespace {
std::string modeName(mmind::api::UhpSettings::UhpCaptureMode captureMode)
{
    switch (captureMode) {
    case mmind::api::UhpSettings::UhpCaptureMode::Camera1:
        return "Camera1";
    case mmind::api::UhpSettings::UhpCaptureMode::Camera2:
        return "Camera2";
    default:
        return "Merge";
    }
}
} // namespace

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    mmind::api::UhpSettings::UhpCaptureMode captureMode;
    mmind::api::ErrorStatus status;
    status = device.getUhpCaptureMode(captureMode);
    if (status.isOK()) {
        std::string mode = modeName(captureMode);
        std::cout << "Capture mode before setting: " << mode << std::endl;

        // Set the capture mode to "Merge".
        captureMode = mmind::api::UhpSettings::UhpCaptureMode::Merge;
        showError(device.setUhpCaptureMode(captureMode));

        showError(device.getUhpCaptureMode(captureMode));
        mode = modeName(captureMode);
        std::cout << "Capture mode after setting: " << mode << std::endl;
    } else
        showError(status);

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
