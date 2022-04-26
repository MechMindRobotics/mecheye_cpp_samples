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
    std::cout << "Disconnect Mech-Eye Success." << std::endl;
    return 0;
}
