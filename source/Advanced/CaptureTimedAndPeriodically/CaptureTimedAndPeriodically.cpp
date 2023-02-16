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

#include <chrono>
#include <iostream>
#include <thread>

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include "CaptureUtil.h"

int main()
{
    const auto captureTime = std::chrono::minutes(5);
    const auto capturePeriod = std::chrono::seconds(10);

    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    mmind::api::MechEyeDeviceInfo deviceInfo;
    showError(device.getDeviceInfo(deviceInfo));
    printDeviceInfo(deviceInfo);

    std::cout << "Starting capturing for " << captureTime.count() << " minutes." << std::endl;

    const auto start = std::chrono::high_resolution_clock::now();

    while (std::chrono::high_resolution_clock::now() - start < captureTime) {
        const auto before = std::chrono::high_resolution_clock::now();

        std::ostringstream ss;
        ss << (std::chrono::duration_cast<std::chrono::seconds>(before - start)).count();
        std::string time = ss.str();

        capture(device, time);

        const auto after = std::chrono::high_resolution_clock::now();
        const auto timeUsed = after - before;
        if (timeUsed < capturePeriod)
            std::this_thread::sleep_for(capturePeriod - timeUsed);
        else
            std::cout << "Your capture time is longer than your capture period. Please increase "
                         "your capture period."
                      << std::endl;

        const auto timeRemaining =
            captureTime - (std::chrono::high_resolution_clock::now() - start);
        const auto remainingMinutes =
            std::chrono::duration_cast<std::chrono::minutes>(timeRemaining);
        const auto remainingSeconds =
            std::chrono::duration_cast<std::chrono::seconds>(timeRemaining - remainingMinutes);
        std::cout << "Remaining time: " << remainingMinutes.count() << " minutes and "
                  << remainingSeconds.count() << "seconds." << std::endl;
    }

    std::cout << "Capturing completed for " << captureTime.count() << " minutes." << std::endl;

    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
