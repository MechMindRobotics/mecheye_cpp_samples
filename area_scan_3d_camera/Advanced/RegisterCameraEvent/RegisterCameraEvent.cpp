/*******************************************************************************
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
With this sample, you can define and register the callback function for monitoring camera events.
*/

#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>
#include "area_scan_3d_camera/CameraEvent.h"
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/api_util.h"

int main()
{
    mmind::eye::Camera camera;
    if (!findAndConnect(camera))
        return -1;

    // Define the callback function for camera events
    auto callbackWithPUser = [](const mmind::eye::EventData* eventDataPtr, const void* extraPayload,
                                void* pUser) {
        const auto occurMs = static_cast<std::chrono::milliseconds>(eventDataPtr->timestamp);
        std::time_t time_occur = static_cast<std::time_t>(
            std::chrono::duration_cast<std::chrono::seconds>(occurMs).count());
        std::tm* tm_occur = std::localtime(&time_occur);
        std::stringstream stream;
        stream << std::put_time(tm_occur, "%Y-%m-%d %X");

        std::cout << "A camera event has occurred." << std::endl
                  << "\tEvent ID:  " << eventDataPtr->eventId << std::endl
                  << "\tFrame ID:  " << eventDataPtr->frameId << std::endl
                  << "\tTimestamp: " << stream.str() << "." << eventDataPtr->timestamp % 1000
                  << std::endl;
    };

    const auto callback =
        std::bind(callbackWithPUser, std::placeholders::_1, std::placeholders::_2, nullptr);

    std::cout << "Register the callback function for the following event:"
              << static_cast<int>(mmind::eye::CameraEvent::Event::CAMERA_EVENT_DISCONNECTED)
              << std::endl;
    // Register the callback function for the CAMERA_EVENT_DISCONNECTED event
    showError(mmind::eye::CameraEvent::registerCameraEventCallback(
        camera, mmind::eye::CameraEvent::CAMERA_EVENT_DISCONNECTED, callback));

    // Let the program sleep for 20 seconds. During this period, if the camera disconnects, the
    // callback function will detect and report the disconnection. To test the event mechanism, you
    // can disconnect the camera Ethernet cable during this period.
    std::cout << "Wait for 20 seconds for disconnect event." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(20000));

    std::cout << "Register the callback function for the following event:"
              << static_cast<int>(mmind::eye::CameraEvent::Event::CAMERA_EVENT_EXPOSURE_END)
              << std::endl;
    showError(mmind::eye::CameraEvent::registerCameraEventCallback(
        camera, mmind::eye::CameraEvent::Event::CAMERA_EVENT_EXPOSURE_END, callback));

    if (!confirmCapture3D()) {
        camera.disconnect();
        return 0;
    }

    // If the 3D data has been acquired successfully, the callback function will detect the
    // CAMERA_EVENT_EXPOSURE_END event.
    // Note: This event is sent after the structured-light projection has been completed. To ensure
    // both 2D and 3D data have been acquired before the event is sent, check the following
    // recommendations: If the flash exposure mode is used for acquiring the 2D data, and the
    // mmind::eye::scanning2d_setting::FlashAcquisitionMode parameter is set to "Fast", call
    // mmind::eye::Camera::capture3D() before calling mmind::eye::Camera::capture2D(). Otherwise,
    // call capture2D() before calling capture3D(). Alternatively, you can call
    // mmind::eye::Camera::capture2Dand3D() instead to avoid the timing issue.
    unsigned row = 0;
    unsigned col = 0;

    mmind::eye::Frame3D frame3D;
    showError(camera.capture3D(frame3D));
    mmind::eye::DepthMap depthMap = frame3D.getDepthMap();
    std::cout << "The size of the depth map is: " << depthMap.width() << " (width) * "
              << depthMap.height() << " (height)." << std::endl;
    try {
        mmind::eye::PointZ depthElem = depthMap.at(row, col);
        std::cout << "The depth value of the pixel at (" << row << ", " << col << ") is "
                  << depthElem.z << " mm." << std::endl;
    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        camera.disconnect();
        return 0;
    }
    // Unregister the callback function.
    // The callback functions are automatically unregistered when the program is terminated.
    std::cout << "Unregister the callback function for the following event: "
              << static_cast<int>(mmind::eye::CameraEvent::Event::CAMERA_EVENT_EXPOSURE_END)
              << std::endl;
    showError(mmind::eye::CameraEvent::unregisterCameraEventCallback(
        camera, mmind::eye::CameraEvent::Event::CAMERA_EVENT_EXPOSURE_END));

    camera.disconnect();
    std::cout << "Disconnected from the camera successfully." << std::endl;
    return 0;
}
