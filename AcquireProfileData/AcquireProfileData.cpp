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
This is a simple example of how to find and connect an available Mech-Eye LNX Device
and then capture a batch line data with trigger source of Encoder.
*/

#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include "profiler/Profiler.h"
#include "profiler/api_util.h"
#include "profiler/parameters/RawImageParameters.h"
#include "profiler/parameters/ScanParameters.h"
#include "profiler/parameters/ProfileExtractionParameters.h"
#include "profiler/parameters/ProfileProcessingParameters.h"

namespace {

void saveMap(const mmind::eye::ProfileBatch& batch, int lineCount, int width,
             const std::string& path)
{
    cv::imwrite(path, cv::Mat(lineCount, width, CV_32FC1, batch.getDepthMap().data()));
}

void saveIntensity(const mmind::eye::ProfileBatch& batch, int lineCount, int width,
                   const std::string& path)
{
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
    // Set the exposure mode to Timed
    showError(currentUserSet.setEnumValue(
        mmind::eye::brightness_settings::ExposureMode::name,
        static_cast<int>(mmind::eye::brightness_settings::ExposureMode::Value::Timed)));

    // Set the exposure time to 100 μs
    showError(currentUserSet.setIntValue(mmind::eye::brightness_settings::ExposureTime::name, 100));

    /*The other option for the exposure mode is HDR, in which three exposure times and the
    y-coordinates of two knee points must be set.
    /*The code for setting the relevant parameters for the HDR exposure mode is given in the
    following notes. */

    /*Set the the exposure sequence for the HDR exposure mode. The exposure sequence contains three
     * exposure times, 100, 10, and 4. The total exposure time is the sum of the three exposure
     * times, which is 114 in this case.*/
    /*The y-coordinates of the two knee points are set to 10 and 60.*/

    // showError(currentUserSet.setEnumValue(
    //      mmind::eye::brightness_settings::ExposureMode::name,
    //      static_cast<int>(mmind::eye::brightness_settings::ExposureMode::Value::HDR)));
    //// Set the exposure time to 100 μs
    // showError(currentUserSet.setIntValue(mmind::eye::brightness_settings::ExposureTime::name,
    // 100)); showError(currentUserSet.setFloatValue(
    //      mmind::eye::brightness_settings::​HdrExposureTimeProportion1::name, 40));
    // showError(currentUserSet.setFloatValue(
    //      mmind::eye::brightness_settings::​HdrExposureTimeProportion2::name, 80));
    ////Set HDR dual slope to 10
    // showError(currentUserSet.setFloatValue(mmind::eye::brightness_settings::HdrFirstThreshold​::name,
    // 10));
    ////Set HDR triple slope to 60
    // showError(currentUserSet.setFloatValue(mmind::eye::brightness_settings::HdrSecondThreshold::name,
    // 60));

    //// Set the trigger source to FixedRate
    // showError(currentUserSet.setEnumValue(
    //      mmind::eye::scan_parameters::LineScanTriggerSource::name,
    //      static_cast<int>(mmind::eye::scan_parameters::LineScanTriggerSource::Value::FixedRate)));

    // Set the trigger source to Encoder
    showError(currentUserSet.setEnumValue(
        mmind::eye::trigger_settings::LineScanTriggerSource::name,
        static_cast<int>(mmind::eye::trigger_settings::LineScanTriggerSource::Value::Encoder)));
    // Set the encoder output mode to Motion
    showError(currentUserSet.setEnumValue(
        mmind::eye::trigger_settings::EncoderTriggerDirection::name,
        static_cast<int>(mmind::eye::trigger_settings::EncoderTriggerDirection::Value::Both)));
    // Set the encoder multiplier to Multiple_1
    showError(currentUserSet.setEnumValue(
        mmind::eye::trigger_settings::EncoderTriggerSignalCountingMode::name,
        static_cast<int>(
            mmind::eye::trigger_settings::EncoderTriggerSignalCountingMode::Value::Multiple_1)));
    // Set the encoder trigger divider value to 10
    showError(
        currentUserSet.setIntValue(mmind::eye::trigger_settings::EncoderTriggerInterval::name, 10));
    // Set the maximum number of lines to be scanned to 1600
    showError(currentUserSet.setIntValue(mmind::eye::scan_settings::ScanLineCount::name, 1600));
    // Set the laser power level to 100
    showError(currentUserSet.setIntValue(mmind::eye::brightness_settings::LaserPower::name, 100));

    mmind::eye::ProfilerInfo profilerInfo;
    if (profiler.getProfilerInfo(profilerInfo).isOK()) {
        // Set the analog gain to 1.3
        if (profilerInfo.model == "Mech-Eye LNX 8030") {
            showError(currentUserSet.setEnumValue(
                mmind::eye::brightness_settings::AnalogGainFor8030::name,
                static_cast<int>(
                    mmind::eye::brightness_settings::AnalogGainFor8030::Value::Gain_1_3)));
        } else {
            showError(currentUserSet.setEnumValue(
                mmind::eye::brightness_settings::AnalogGain::name,
                static_cast<int>(mmind::eye::brightness_settings::AnalogGain::Value::Gain_1_3)));
        }
    }

    showError(currentUserSet.setIntValue(mmind::eye::brightness_settings::DigitalGain::name, 0));
    // Set the grayscale value threshold to 50
    showError(
        currentUserSet.setIntValue(mmind::eye::profile_extraction::MinGrayscaleValue::name, 50));
    // Set the minimun laser line width to 2
    showError(
        currentUserSet.setIntValue(mmind::eye::profile_extraction::MinLaserLineWidth::name, 2));
    // Set the maximum laser line width to 20
    showError(
        currentUserSet.setIntValue(mmind::eye::profile_extraction::MaxLaserLineWidth::name, 20));
    // Set the minimum laser line intensity to 10
    showError(
        currentUserSet.setIntValue(mmind::eye::profile_extraction::MinSpotIntensity::name, 51));
    // Set the maximum laser line intensity to 205
    showError(
        currentUserSet.setIntValue(mmind::eye::profile_extraction::MaxSpotIntensity::name, 205));
    /* Set the maximum number of invalid points to be interpolated to 16. If the number of continous
     * invalid points is less than or equal to 16, these points will be filled */
    showError(currentUserSet.setIntValue(mmind::eye::profile_processing::GapFilling::name, 16));
    // Set the profile extraction strategy to Strongest
    showError(currentUserSet.setEnumValue(
        mmind::eye::profile_extraction::SpotSelection::name,
        static_cast<int>(mmind::eye::profile_extraction::SpotSelection::Value::Strongest)));
    /* Set the filter type to Mean. When the filter type is set to Mean or
     * MeanEdgePreserving, setLnxMeanFilterWindow can be called to set the window size for
     * mean filtering. When the filter type is set to Median,
     * setLnxMedianFilterWindow can be called to set the window size for median filtering.*/
    showError(currentUserSet.setEnumValue(
        mmind::eye::profile_processing::Filter::name,
        static_cast<int>(mmind::eye::profile_processing::Filter::Value::Mean)));
    // Set the window size for mean filtering to WindowSize_2
    showError(currentUserSet.setEnumValue(
        mmind::eye::profile_processing::MeanFilterWindowSize::name,
        static_cast<int>(
            mmind::eye::profile_processing::MeanFilterWindowSize::Value::WindowSize_2)));

    int dataWidth = 0;
    // Get the line width in the X direction
    showError(currentUserSet.getIntValue(mmind::eye::scan_settings::DataPointsPerProfile::name,
                                         dataWidth));
    int captureLineCount = 0;
    // Get the current maximum number of lines to be scanned
    currentUserSet.getIntValue(mmind::eye::scan_settings::ScanLineCount::name, captureLineCount);

    const int size = captureLineCount * dataWidth;

    std::cout << "Start scanning!" << std::endl;

    // Start scanning
    mmind::eye::ProfileBatch totalBatch(dataWidth);

    if (profiler.startAcquisition().isOK() && profiler.triggerSoftware().isOK()) {
        totalBatch.reserve(captureLineCount);
        while (totalBatch.height() < captureLineCount) {
            // Get image data
            mmind::eye::ProfileBatch batch(dataWidth);
            mmind::eye::ErrorStatus status = profiler.retrieveBatchData(batch);
            if (status.isOK()) {
                totalBatch.append(batch);
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            } else {
                showError(status);
                break;
            }
        }
    }
    std::cout << "Save depth map and intensity image." << std::endl;
    saveMap(totalBatch, captureLineCount, dataWidth, "Depth.tif");
    saveIntensity(totalBatch, captureLineCount, dataWidth, "Intensity.tif");

    // Disconnect from the camera
    profiler.disconnect();
    return 0;
}
