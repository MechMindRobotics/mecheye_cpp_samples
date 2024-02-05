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
With this sample, you can acquire the profile data using a callback function, generate the intensity
image and depth map, and save the images.
*/

#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include "profiler/Profiler.h"
#include "profiler/api_util.h"
#include "profiler/parameters/ProfileProcessingParameters.h"
#include "profiler/parameters/ProfileExtractionParameters.h"
#include "profiler/parameters/RawImageParameters.h"
#include "profiler/parameters/ScanParameters.h"

namespace {
std::mutex kMutex;

// Define the callback function for retrieving the profile data
void callbackFunc(const mmind::eye::ProfileBatch& batch, void* pUser)
{
    std::unique_lock<std::mutex> lock(kMutex);
    auto* outPutBatch = static_cast<mmind::eye::ProfileBatch*>(pUser);
    outPutBatch->append(batch);
}

void saveMap(const mmind::eye::ProfileBatch& batch, int lineCount, int width,
             const std::string& path)
{
    if (batch.isEmpty()) {
        std::cout
            << "The depth map cannot be saved because the batch does not contain any profile data."
            << std::endl;
        return;
    }
    cv::imwrite(path, cv::Mat(lineCount, width, CV_32FC1, batch.getDepthMap().data()));
}

void saveIntensity(const mmind::eye::ProfileBatch& batch, int lineCount, int width,
                   const std::string& path)
{
    if (batch.isEmpty()) {
        std::cout
            << "The intensity cannot be saved because the batch does not contain any profile data."
            << std::endl;
        return;
    }
    cv::imwrite(path, cv::Mat(lineCount, width, CV_8UC1, batch.getIntensityImage().data()));
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

    mmind::eye::UserSet currentUserSet = profiler.currentUserSet();
    // Set the "Exposure Mode" parameter to "Timed"
    showError(currentUserSet.setEnumValue(
        mmind::eye::brightness_settings::ExposureMode::name,
        static_cast<int>(mmind::eye::brightness_settings::ExposureMode::Value::Timed)));

    // Set the "Exposure Time" parameter to 100 μs
    showError(currentUserSet.setIntValue(mmind::eye::brightness_settings::ExposureTime::name, 100));

    /*You can also use the HDR exposure mode, in which the laser profiler exposes in three phases
    while acquiring one profile. In this mode, you need to set the total exsposure time, the
    proportions of the three exposure phases, as well as the two thresholds of grayscale values. The
    code for setting the relevant parameters for the HDR exposure mode is given in the following
    comments.*/

    // Set the "Exposure Mode" parameter to "HDR"
    // showError(currentUserSet.setEnumValue(
    //     mmind::eye::brightness_settings::ExposureMode::name,
    //     static_cast<int>(mmind::eye::brightness_settings::ExposureMode::Value::HDR)));

    // Set the total exposure time to 100 μs
    // showError(currentUserSet.setIntValue(mmind::eye::brightness_settings::ExposureTime::name,
    // 100));

    // Set the proportion of the first exposure phase to 40%
    // showError(currentUserSet.setFloatValue(
    //     mmind::eye::brightness_settings::HdrExposureTimeProportion1::name, 40));

    // Set the proportion of the first + second exposure phases to 80% (that is, the second exposure
    // phase occupies 40%, and the third exposure phase occupies 20% of the total exposure time)
    // showError(currentUserSet.setFloatValue(
    //     mmind::eye::brightness_settings::HdrExposureTimeProportion2::name, 80));

    // Set the first threshold to 10. This limits the maximum grayscale value to 10 after the first
    // exposure phase is completed. showError(
    //     currentUserSet.setFloatValue(mmind::eye::brightness_settings::HdrFirstThreshold::name,
    //     10));

    // Set the second threshold to 60. This limits the maximum grayscale value to 60 after the
    // second exposure phase is completed. showError(currentUserSet.setFloatValue(
    //     mmind::eye::brightness_settings::HdrSecondThreshold::name, 60));

    // Set the "Data Acquisition Trigger Source" parameter to "Software"
    showError(currentUserSet.setEnumValue(
        mmind::eye::trigger_settings::DataAcquisitionTriggerSource::name,
        static_cast<int>(
            mmind::eye::trigger_settings::DataAcquisitionTriggerSource::Value::Software)));

    // Set the "Data Acquisition Trigger Source" parameter to "External"
    // showError(currentUserSet.setEnumValue(
    //     mmind::eye::trigger_settings::DataAcquisitionTriggerSource::name,
    //     static_cast<int>(mmind::eye::trigger_settings::DataAcquisitionTriggerSource::Value::External)));

    // Set the "Line Scan Trigger Source" parameter to "Fixed rate"
    showError(currentUserSet.setEnumValue(
        mmind::eye::trigger_settings::LineScanTriggerSource::name,
        static_cast<int>(mmind::eye::trigger_settings::LineScanTriggerSource::Value::FixedRate)));
    // Set the "Software Trigger Rate" to 1000 Hz
    showError(currentUserSet.setFloatValue(mmind::eye::trigger_settings::SoftwareTriggerRate::name,
                                           1000));
    // Set the "Scan Line Count" parameter (the number of lines to be scanned) to 1600
    showError(currentUserSet.setIntValue(mmind::eye::scan_settings::ScanLineCount::name, 1600));
    // Set the timeout for retrieving profile data by callback to 4000 ms. The timeout means that
    // the maximum time to wait for retrieving profile data fully, so that it needs to be set
    // according to the trigger rate. If the timeout occurs and profile data are not yet fully
    // retrieved, the missing profiles will be filled with invalid data.
    showError(currentUserSet.setIntValue(mmind::eye::scan_settings::CallbackRetrievalTimeout::name,
                                         4000));
    // Set the "Laser Power" parameter to 100
    showError(currentUserSet.setIntValue(mmind::eye::brightness_settings::LaserPower::name, 100));

    // Set the "Analog Gain" parameter to "Gain_2"
    showError(currentUserSet.setEnumValue(
        mmind::eye::brightness_settings::AnalogGain::name,
        static_cast<int>(mmind::eye::brightness_settings::AnalogGain::Value::Gain_2)));

    // Set the "Digital Gain" parameter to 0
    showError(currentUserSet.setIntValue(mmind::eye::brightness_settings::DigitalGain::name, 0));
    // Set the "Minimum Grayscale Value" parameter to 50
    showError(
        currentUserSet.setIntValue(mmind::eye::profile_extraction::MinGrayscaleValue::name, 50));
    // Set the "Minimum Laser Line Width" parameter to 2
    showError(
        currentUserSet.setIntValue(mmind::eye::profile_extraction::MinLaserLineWidth::name, 2));
    // Set the "Maximum Laser Line Width" parameter to 20
    showError(
        currentUserSet.setIntValue(mmind::eye::profile_extraction::MaxLaserLineWidth::name, 20));
    // This parameter is only effective for firmware 2.2.1 and below. For firmware 2.3.0 and above,
    // adjustment of this parameter does not take effect.
    // Set the "Minimum Spot Intensity" parameter to 51
    showError(
        currentUserSet.setIntValue(mmind::eye::profile_extraction::MinSpotIntensity::name, 51));
    // This parameter is only effective for firmware 2.2.1 and below. For firmware 2.3.0 and above,
    // adjustment of this parameter does not take effect.
    // Set the "Maximum Spot Intensity" parameter to 205
    showError(
        currentUserSet.setIntValue(mmind::eye::profile_extraction::MaxSpotIntensity::name, 205));
    /* Set the "Gap Filling" parameter to 16, which controls the size of the gaps that can be filled
     * in the profile. When the number of consecutive data points in a gap in the profile is no
     * greater than 16, this gap will be filled. */
    showError(currentUserSet.setIntValue(mmind::eye::profile_processing::GapFilling::name, 16));
    // Set the "Spot Selection" parameter to "Strongest"
    showError(currentUserSet.setEnumValue(
        mmind::eye::profile_extraction::SpotSelection::name,
        static_cast<int>(mmind::eye::profile_extraction::SpotSelection::Value::Strongest)));

    /* Set the "Filter" parameter to "Mean". The "Mean Filter Window Size" parameter needs to be set
     * as well. This parameter controls the window size of mean filter. If the "Filter" parameter is
     * set to "Median", the "Median Filter Window Size" parameter needs to be set. This parameter
     * controls the window size of median filter.*/
    showError(currentUserSet.setEnumValue(
        mmind::eye::profile_processing::Filter::name,
        static_cast<int>(mmind::eye::profile_processing::Filter::Value::Mean)));
    // Set the "Mean Filter Window Size" parameter to 2
    showError(currentUserSet.setEnumValue(
        mmind::eye::profile_processing::MeanFilterWindowSize::name,
        static_cast<int>(
            mmind::eye::profile_processing::MeanFilterWindowSize::Value::WindowSize_2)));

    int dataPoints = 0;
    // Get the number of data points in each profile
    showError(currentUserSet.getIntValue(mmind::eye::scan_settings::DataPointsPerProfile::name,
                                         dataPoints));
    int captureLineCount = 0;
    // Get the current value of the "Scan Line Count" parameter
    currentUserSet.getIntValue(mmind::eye::scan_settings::ScanLineCount::name, captureLineCount);

    // Define a ProfileBatch object to store the profile data
    mmind::eye::ProfileBatch profileBatch(dataPoints);
    // Register the callback function
    auto status = profiler.registerAcquisitionCallback(callbackFunc, &profileBatch);
    if (!status.isOK()) {
        showError(status);
        return -1;
    }

    // Call the startAcquisition to take the laser profiler into the acquisition ready status
    status = profiler.startAcquisition();
    if (!status.isOK()) {
        showError(status);
        return -1;
    }

    // Call the triggerSoftware() to start the data acquisition
    status = profiler.triggerSoftware();
    if (!status.isOK()) {
        showError(status);
        return -1;
    }

    while (true) {
        std::unique_lock<std::mutex> lock(kMutex);
        if (profileBatch.isEmpty()) {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else
            break;
    }

    saveMap(profileBatch, profileBatch.height(), profileBatch.width(), "DepthMap.tif");

    // Uncomment the following line to save a virtual device file using the ProfileBatch
    // profileBatch acquired.
    // profiler.saveVirtualDeviceFile(profileBatch, "test.mraw");

    profileBatch.clear();

    // Disconnect from the laser profiler
    profiler.disconnect();
    return 0;
}
