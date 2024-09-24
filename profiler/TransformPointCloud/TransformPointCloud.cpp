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


/*
 * Calculate the initial coordinates of each point, apply the rigid body transformations to the
 * initial coordinates, and then write the transformed coordinates to the PLY file.
 */
bool transformAndSaveToPLY(float* data, int* yValues, int captureLineCount, int dataWidth, float xUnit,
                   float yUnit, const std::string& fileName, bool isOrganized,
                   const mmind::eye::FrameTransformation& coordTransformation)
{
    FILE* fp = fopen(fileName.c_str(), "w");

    if (!fp)
        return false;

    unsigned validPointCount{0};
    if (!isOrganized) {
        for (int y = 0; y < captureLineCount; ++y) {
            for (int x = 0; x < dataWidth; ++x) {
                if (!std::isnan(data[y * dataWidth + x]))
                    validPointCount++;
            }
        }
    }

    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "comment File generated\n");
    fprintf(fp, "comment x y z data unit in mm\n");
    fprintf(fp, "element vertex %u\n",
            isOrganized ? static_cast<unsigned>(captureLineCount * dataWidth) : validPointCount);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "end_header\n");

    const auto& rot = coordTransformation.rotation;
    const auto& trans = coordTransformation.translation;
    for (int y = 0; y < captureLineCount; ++y) {
        for (int x = 0; x < dataWidth; ++x) {
            if (!std::isnan(data[y * dataWidth + x])) {
                // Calculate the initial coordinates of each point from the original profile data.
                const auto xPos = x * xUnit * kPitch;
                const auto yPos = yValues[y] * yUnit * kPitch;
                const auto zPos = data[y * dataWidth + x];
                // Apply the rigid body transformations to the initial coordinates to obtain the
                // coordinates in the custom reference frame.
                const auto transformedX =
                    xPos * rot[0][0] + yPos * rot[0][1] + zPos * rot[0][2] + trans[0];
                const auto transformedY =
                    xPos * rot[1][0] + yPos * rot[1][1] + zPos * rot[1][2] + trans[1];
                const auto transformedZ =
                    xPos * rot[2][0] + yPos * rot[2][1] + zPos * rot[2][2] + trans[2];
                fprintf(fp, "%f %f %f\n", static_cast<float>(transformedX),
                        static_cast<float>(transformedY), static_cast<float>(transformedZ));
            } else if (isOrganized)
                fprintf(fp, "nan nan nan\n");
        }
    }

    fclose(fp);
    return true;
}
// Convert the profile data to an untextured point cloud in the custom reference frame and save it
// to a PLY file.
void convertBatchToPointCloudWithTransformation(mmind::eye::ProfileBatch& batch, const mmind::eye::UserSet& userSet,
                               const mmind::eye::FrameTransformation& coordinateTransformation)
{
    if (batch.isEmpty())
        return;

    // Get the X-axis resolution
    double xUnit{};
    auto status =
        userSet.getFloatValue(mmind::eye::point_cloud_resolutions::XAxisResolution::name, xUnit);
    if (!status.isOK()) {
        showError(status);
        return;
    }

    // Get the Y resolution
    double yUnit{};
    status = userSet.getFloatValue(mmind::eye::point_cloud_resolutions::YResolution::name, yUnit);
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
    //         yUnit = atoi(str.c_str());
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

    // Shift the encoder values around zero
    std::vector<int> encoderVals;
    encoderVals.reserve(batch.height());
    auto encoder = batch.getEncoderArray();
    for (int r = 0; r < batch.height(); ++r)
        encoderVals.push_back(
            useEncoderValues ? shiftEncoderValsAroundZero(encoder[r], encoder[0]) / triggerInterval
                             : r);

    std::cout << "Save the transformed point cloud." << std::endl;
    transformAndSaveToPLY(batch.getDepthMap().data(), encoderVals.data(), batch.height(), batch.width(),
                  xUnit, yUnit, "PointCloud.ply", true, coordinateTransformation);
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
     * The custom reference frame can be adjusted using the "Custom Reference Frame" tool in Mech-Eye
     * Viewer. The rigid body transformations are automatically calculated.
     * Alternatively, you can avoid using the current interface and instead use the rotation and
     * translation methods of FrameTransformation to construct a transformation matrix manually.
     * However, this method is not recommended because it is less intuitive.
     */
    const auto transformation = getTransformationParams(profiler);
    if (!transformation.isValid()) {
        std::cout << "Transformation parameters are not set. Please configure the transformation "
                     "parameters using the custom coordinate system tool in the client." << std::endl;
    }
    // Transform the reference frame, generate the untextured point cloud, and save the point cloud
    convertBatchToPointCloudWithTransformation(profileBatch, userSet, transformation);

    // Disconnect from the laser profiler
    profiler.disconnect();
    return 0;
}
