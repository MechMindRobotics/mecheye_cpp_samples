/*******************************************************************************
 *BSD 3-Clause License
 *
 *Copyright (c) 2016-2025, Mech-Mind Robotics
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
With this sample, you can define and register the callback function for monitoring the profiler
connection status and line trigger missed event.
*/

#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>
#include "profiler/Profiler.h"
#include "profiler/ProfilerEvent.h"
#include "profiler/api_util.h"

bool acquireProfileData(mmind::eye::Profiler& profiler, mmind::eye::ProfileBatch& totalBatch,
                        int captureLineCount, int dataWidth, bool isSoftwareTrigger)
{
    /* Call startAcquisition() to set the laser profiler to acquisition-ready status, and
    then call triggerSoftware() to start the data acquisition (triggered by software).*/
    std::cout << "Start data acquisition." << std::endl;
    auto status = profiler.startAcquisition();
    if (!status.isOK()) {
        showError(status);
        return false;
    }

    if (isSoftwareTrigger) {
        status = profiler.triggerSoftware();
        if (!status.isOK()) {
            showError(status);
            return false;
        }
    }

    totalBatch.clear();
    totalBatch.reserve(captureLineCount);
    while (totalBatch.height() < captureLineCount) {
        // Retrieve the profile data
        mmind::eye::ProfileBatch batch(dataWidth);
        status = profiler.retrieveBatchData(batch);
        if (status.isOK()) {
            if (!totalBatch.append(batch))
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } else {
            showError(status);
            return false;
        }
    }

    std::cout << "Stop data acquisition." << std::endl;
    status = profiler.stopAcquisition();
    if (!status.isOK())
        showError(status);
    return status.isOK();
}

int main()
{
    mmind::eye::Profiler profiler;
    if (!findAndConnect(profiler))
        return -1;

    // Define the callback function for handling the events
    auto callbackWithPUser = [](const mmind::eye::ProfilerEvent::EventData* eventDataPtr,
                                const void* extraPayload, void* pUser) {
        const auto occurMs = static_cast<std::chrono::milliseconds>(eventDataPtr->timestamp);
        std::time_t time_occur = static_cast<std::time_t>(
            std::chrono::duration_cast<std::chrono::seconds>(occurMs).count());
        std::tm* tm_occur = std::localtime(&time_occur);
        std::stringstream stream;
        stream << std::put_time(tm_occur, "%Y-%m-%d %X");

        std::cout << "A profiler event has occurred." << std::endl
                  << "\tEvent ID:  " << eventDataPtr->eventId << std::endl
                  << "\tEvent Name: " << eventDataPtr->eventName << std::endl
                  << "\tTimestamp: " << stream.str() << "." << eventDataPtr->timestamp % 1000
                  << std::endl;

        if (!extraPayload)
            return;
        const auto& payload =
            *reinterpret_cast<const mmind::eye::ProfilerEvent::Payload*>(extraPayload);
        for (const auto& member : payload) {
            std::cout << "\t" << member.name << ": ";
            switch (member.type) {
            case mmind::eye::ProfilerEvent::PayloadMember::Type::_UInt32:
                std::cout << member.value.uint32Value;
                break;
            case mmind::eye::ProfilerEvent::PayloadMember::Type::_Int32:
                std::cout << member.value.int32Value;
                break;
            case mmind::eye::ProfilerEvent::PayloadMember::Type::_Int64:
                std::cout << member.value.int64Value;
                break;
            case mmind::eye::ProfilerEvent::PayloadMember::Type::_Float:
                std::cout << member.value.floatValue;
                break;
            case mmind::eye::ProfilerEvent::PayloadMember::Type::_Double:
                std::cout << member.value.doubleValue;
                break;
            case mmind::eye::ProfilerEvent::PayloadMember::Type::_Bool:
                std::cout << member.value.boolValue;
                break;
            case mmind::eye::ProfilerEvent::PayloadMember::Type::_String:
                std::cout << member.value.stringValue;
                break;
            }
            std::cout << std::endl;
        }
    };

    const auto callback =
        std::bind(callbackWithPUser, std::placeholders::_1, std::placeholders::_2, nullptr);

    // Set the heartbeat interval to 2 seconds
    profiler.setHeartbeatInterval(2000);

    std::vector<mmind::eye::ProfilerEvent::EventInfo> supportedEvents;
    showError(mmind::eye::ProfilerEvent::getSupportedEvents(profiler, supportedEvents));
    std::cout << "\nEvents supported by this profiler\n";
    for (const auto& event : supportedEvents) {
        std::cout << "\n" << event.eventName << ": " << event.eventId << "\n";
        std::cout << "Register the callback function for the event " << event.eventName << ":\n";
        showError(mmind::eye::ProfilerEvent::registerProfilerEventCallback(profiler, event.eventId,
                                                                           callback));
    }
    std::cout << std::endl;

    // The program pauses for 20 seconds to allow the user to test if the profiler disconnection
    // event works properly. If the network cable is unplugged, the disconnection will be detected
    // and the callback function will be triggered.
    std::cout << "The program pauses for 20 seconds for testing the disconnection event."
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(20000));

    if (!confirmCapture()) {
        profiler.disconnect();
        return -1;
    }

    auto userSet = profiler.currentUserSet();

    int dataWidth = 0;
    // Get the number of data points in each profile
    showError(
        userSet.getIntValue(mmind::eye::scan_settings::DataPointsPerProfile::name, dataWidth));
    int captureLineCount = 0;
    // Get the current value of the "Scan Line Count" parameter
    userSet.getIntValue(mmind::eye::scan_settings::ScanLineCount::name, captureLineCount);

    // Define a ProfileBatch object to store the profile data
    mmind::eye::ProfileBatch profileBatch(dataWidth);

    int dataAcquisitionTriggerSource{};
    showError(userSet.getEnumValue(mmind::eye::trigger_settings::DataAcquisitionTriggerSource::name,
                                   dataAcquisitionTriggerSource));
    bool isSoftwareTrigger =
        dataAcquisitionTriggerSource ==
        static_cast<int>(
            mmind::eye::trigger_settings::DataAcquisitionTriggerSource::Value::Software);

    // Acquire profile data without using callback
    if (!acquireProfileData(profiler, profileBatch, captureLineCount, dataWidth, isSoftwareTrigger))
        return -1;

    std::cout << "Unregister the callback function for the following event: "
              << supportedEvents.front().eventName << std::endl;
    showError(mmind::eye::ProfilerEvent::unregisterProfilerEventCallback(
        profiler, supportedEvents.front().eventId));

    profiler.disconnect();
    std::cout << "Disconnected from the profiler successfully." << std::endl;
    return 0;
}
