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
With this sample, you can retrieve point clouds in the custom reference frame.
*/

#include <iostream>
#include <thread>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <mutex>
#include "profiler/Profiler.h"
#include "profiler/api_util.h"
#include "profiler/parameters/ScanParameters.h"
#include "profiler/PointCloudTransformation.h"

namespace {
std::mutex kMutex;
bool acquireProfileData(mmind::eye::Profiler& profiler, mmind::eye::ProfileBatch& totalBatch,
                        int captureLineCount, int dataWidth, bool isSoftwareTrigger)
{
    /* Call startAcquisition() to enter the laser profiler into the acquisition ready status, and
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

void setParameters(mmind::eye::UserSet& userSet)
{
    // Set the "Data Acquisition Trigger Source" parameter to "Software"
    showError(userSet.setEnumValue(
        mmind::eye::trigger_settings::DataAcquisitionTriggerSource::name,
        static_cast<int>(
            mmind::eye::trigger_settings::DataAcquisitionTriggerSource::Value::Software)));

    // Set the "Line Scan Trigger Source" parameter to "Fixed rate"
    showError(userSet.setEnumValue(
        mmind::eye::trigger_settings::LineScanTriggerSource::name,
        static_cast<int>(mmind::eye::trigger_settings::LineScanTriggerSource::Value::FixedRate)));
    // Set the "Software Trigger Rate" to 1000 Hz
    showError(userSet.setFloatValue(mmind::eye::trigger_settings::SoftwareTriggerRate::name, 1000));

    // Set the "Scan Line Count" parameter (the number of lines to be scanned) to 1600
    showError(userSet.setIntValue(mmind::eye::scan_settings::ScanLineCount::name, 1600));
}

// Convert the profile data to an untextured point cloud in the custom reference frame and save it
// to a PLY file.
void convertBatchToPointCloudWithTransformation(
    mmind::eye::ProfileBatch& batch, const mmind::eye::UserSet& userSet,
    const mmind::eye::FrameTransformation& coordinateTransformation)
{
    if (batch.isEmpty())
        return;

    // Get the X-axis resolution
    double xResolution{};
    auto status = userSet.getFloatValue(mmind::eye::point_cloud_resolutions::XAxisResolution::name,
                                        xResolution);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    // Get the Y resolution
    double yResolution{};
    status =
        userSet.getFloatValue(mmind::eye::point_cloud_resolutions::YResolution::name, yResolution);
    if (!status.isOK()) {
        showError(status);
        return;
    }
    // // Uncomment the following lines for custom Y Unit
    // // Prompt to enter the desired encoder resolution, which is the travel distance corresponding
    // // to
    // // one quadrature signal.
    // std::cout << "Please enter the desired encoder resolution (integer, unit: μm, min: "
    //  "1, max: 65535): ";
    // while (true) {
    //     std::string str;
    //     std::cin >> str;
    //     if (std::regex_match(str.begin(), str.end(), std::regex{"[0-9]+"})) {
    //         yResolution = atoi(str.c_str());
    //         break;
    //     }
    //     std::cout << "Input invalid! Please enter the desired encoder resolution (integer, unit:
    //     "
    //                  "μm, min: 1, max: 65535): ";
    // }

    int lineScanTriggerSource{};
    status = userSet.getEnumValue(mmind::eye::trigger_settings::LineScanTriggerSource::name,
                                  lineScanTriggerSource);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    bool useEncoderValues =
        lineScanTriggerSource ==
        static_cast<int>(mmind::eye::trigger_settings::LineScanTriggerSource::Value::Encoder);

    int triggerInterval{};
    status = userSet.getIntValue(mmind::eye::trigger_settings::EncoderTriggerInterval::name,
                                 triggerInterval);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    std::cout << "Save the transformed point cloud." << std::endl;
    showError(mmind::eye::ProfileBatch::saveUntexturedPointCloud(
        transformPointCloud(coordinateTransformation,
                            batch.getUntexturedPointCloud(xResolution, yResolution,
                                                          useEncoderValues, triggerInterval)),
        mmind::eye::FileFormat::PLY, "PointCloud.ply"));
}
} // namespace

int main()
{
    mmind::eye::Profiler profiler;
    if (!findAndConnect(profiler))
        return -1;

    if (!confirmCapture()) {
        profiler.disconnect();
        return -1;
    }

    mmind::eye::UserSet userSet = profiler.currentUserSet();

    // Set the parameters
    setParameters(userSet);

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

    if (profileBatch.checkFlag(mmind::eye::ProfileBatch::BatchFlag::Incomplete))
        std::cout << "Part of the batch's data is lost, the number of valid profiles is: "
                  << profileBatch.validHeight() << "." << std::endl;

    /*
     * Obtain the rigid body transformation from the camera reference frame to the custom reference
     * frame.
     * The custom reference frame can be adjusted using the "Custom Reference Frame" tool in
     * Mech-Eye Viewer. The rigid body transformations are automatically calculated. Alternatively,
     * you can avoid using the current interface and instead use the rotation and translation
     * methods of FrameTransformation to construct a transformation matrix manually. However, this
     * method is not recommended because it is less intuitive.
     */
    const auto transformation = getTransformationParams(profiler);
    if (!transformation.isValid()) {
        std::cout << "Transformation parameters are not set. Please configure the transformation "
                     "parameters using the custom coordinate system tool in the client."
                  << std::endl;
    }
    // Transform the reference frame, generate the untextured point cloud, and save the point cloud
    convertBatchToPointCloudWithTransformation(profileBatch, userSet, transformation);

    // Disconnect from the laser profiler
    profiler.disconnect();
    return 0;
}
