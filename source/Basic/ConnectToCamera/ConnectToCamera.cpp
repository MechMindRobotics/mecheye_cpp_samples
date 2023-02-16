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

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include <iostream>

int main()
{
    std::cout << "Find Mech-Eye devices..." << std::endl;
    std::vector<mmind::api::MechEyeDeviceInfo> deviceInfoList =
        mmind::api::MechEyeDevice::enumerateMechEyeDeviceList();

    if (deviceInfoList.empty()) {
        std::cout << "No Mech-Eye device found." << std::endl;
        return -1;
    }

    for (int i = 0; i < deviceInfoList.size(); i++) {
        std::cout << "Mech-Eye device index : " << i << std::endl;
        printDeviceInfo(deviceInfoList[i]);
    }

    std::cout << "Please enter the device index you want to connect: ";
    unsigned inputIndex;

    while (1) {
        std::string str;
        std::cin >> str;
        if (isNumber(str) && atoi(str.c_str()) < deviceInfoList.size()) {
            inputIndex = atoi(str.c_str());
            break;
        }
        std::cout << "Input invalid! Please enter the device index you want to connect: ";
    }

    mmind::api::ErrorStatus status;
    mmind::api::MechEyeDevice device;
    status = device.connect(deviceInfoList[inputIndex]);

    if (!status.isOK()) {
        showError(status);
        return -1;
    }

    std::cout << "Connected to the Mech-Eye device successfully." << std::endl;

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
