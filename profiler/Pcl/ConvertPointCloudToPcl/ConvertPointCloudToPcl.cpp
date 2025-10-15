/*******************************************************************************
 *BSD 3-Clause License
 *
 *Copyright (c) 2016-2025, Mech-Mind Robotics Technologies Co., Ltd.
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
With this sample, you can obtain the point cloud data from the profiler and convert it to the PCL
data structure.
*/

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <string>
#include <cstddef>
#include <mutex>
#include <thread>
#include "profiler/Profiler.h"
#include "profiler/api_util.h"
#include "profiler/parameters/ProfileProcessingParameters.h"
#include "profiler/parameters/ProfileExtractionParameters.h"
#include "profiler/parameters/RawImageParameters.h"
#include "profiler/parameters/ScanParameters.h"

namespace {
std::mutex kMutex;

void setTimedExposure(mmind::eye::UserSet& userSet, int exposureTime)
{
    // Set the "Exposure Mode" parameter to "Timed"
    showError(userSet.setEnumValue(
        mmind::eye::brightness_settings::ExposureMode::name,
        static_cast<int>(mmind::eye::brightness_settings::ExposureMode::Value::Timed)));

    // Set the "Exposure Time" parameter to {exposureTime} μs
    showError(
        userSet.setIntValue(mmind::eye::brightness_settings::ExposureTime::name, exposureTime));
}

void setHDRExposure(mmind::eye::UserSet& userSet, int exposureTime, double proportion1,
                    double proportion2, double firstThreshold, double secondThreshold)
{
    // Set the "Exposure Mode" parameter to "HDR"
    showError(userSet.setEnumValue(
        mmind::eye::brightness_settings::ExposureMode::name,
        static_cast<int>(mmind::eye::brightness_settings::ExposureMode::Value::HDR)));

    // Set the total exposure time to {exposureTime} μs
    showError(
        userSet.setIntValue(mmind::eye::brightness_settings::ExposureTime::name, exposureTime));

    // Set the proportion of the first exposure phase to {proportion1}%
    showError(userSet.setFloatValue(
        mmind::eye::brightness_settings::HdrExposureTimeProportion1::name, proportion1));

    // Set the proportion of the first + second exposure phases to {proportion2}% (that is, the
    // second exposure phase occupies {proportion2 - proportion1}%, and the third exposure phase
    // occupies {100 - proportion2}% of the total exposure time)
    showError(userSet.setFloatValue(
        mmind::eye::brightness_settings::HdrExposureTimeProportion2::name, proportion2));

    // Set the first threshold to {firstThreshold}. This limits the maximum grayscale value to
    // {firstThreshold} after the first exposure phase is completed.
    showError(userSet.setFloatValue(mmind::eye::brightness_settings::HdrFirstThreshold::name,
                                    firstThreshold));

    // Set the second threshold to {secondThreshold}. This limits the maximum grayscale value to
    // {secondThreshold} after the second exposure phase is completed.
    showError(userSet.setFloatValue(mmind::eye::brightness_settings::HdrSecondThreshold::name,
                                    secondThreshold));
}

void setParameters(mmind::eye::UserSet& userSet)
{
    // Set the "Exposure Mode" parameter to "Timed"
    // Set the "Exposure Time" parameter to 100 μs
    setTimedExposure(userSet, 100);

    /*You can also use the HDR exposure mode, in which the laser profiler exposes in three phases
    while acquiring one profile. In this mode, you need to set the total exposure time, the
    proportions of the three exposure phases, as well as the two thresholds of grayscale values. The
    code for setting the relevant parameters for the HDR exposure mode is given in the following
    comments.*/
    // // Set the "Exposure Mode" parameter to "HDR"
    // // Set the total exposure time to 100 μs
    // // Set the proportion of the first exposure phase to 40%
    // // Set the proportion of the first + second exposure phases to 80% (that is, the second
    // // exposure phase occupies 40%, and the third exposure phase occupies 20% of the total
    // // exposure
    // // Set the first threshold to 10. This limits the maximum grayscale value to 10 after the
    // // first exposure phase is completed.
    // // Set the second threshold to 60. This limits the maximum grayscale value to 60 after the
    // // second exposure phase is completed.
    // setHDRExposure(userSet, 100, 40, 80, 10, 60);

    // // Enable outlier removal and adjust the outlier removal intensity.
    // // Set the "EnableOutlierRemoval" parameter to true
    // showError(
    //    userSet.setBoolValue(mmind::eye::profile_processing::EnableOutlierRemoval::name, true));
    // // Set the "OutlierRemovalIntensity" parameter to "VeryLow"
    // showError(userSet.setEnumValue(
    //    mmind::eye::profile_processing::OutlierRemovalIntensity::name,
    //    static_cast<int>(
    //        mmind::eye::profile_processing::OutlierRemovalIntensity::Value::VeryLow)));

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

    // Set the "Laser Power" parameter to 100
    showError(userSet.setIntValue(mmind::eye::brightness_settings::LaserPower::name, 100));
    // Set the "Analog Gain" parameter to "Gain_2"
    showError(userSet.setEnumValue(
        mmind::eye::brightness_settings::AnalogGain::name,
        static_cast<int>(mmind::eye::brightness_settings::AnalogGain::Value::Gain_2)));
    // Set the "Digital Gain" parameter to 0
    showError(userSet.setIntValue(mmind::eye::brightness_settings::DigitalGain::name, 0));

    // Set the "Minimum Grayscale Value" parameter to 50
    showError(userSet.setIntValue(mmind::eye::profile_extraction::MinGrayscaleValue::name, 50));
    // Set the "Minimum Laser Line Width" parameter to 2
    showError(userSet.setIntValue(mmind::eye::profile_extraction::MinLaserLineWidth::name, 2));
    // Set the "Maximum Laser Line Width" parameter to 20
    showError(userSet.setIntValue(mmind::eye::profile_extraction::MaxLaserLineWidth::name, 20));
    // Set the "Spot Selection" parameter to "Strongest"
    showError(userSet.setEnumValue(
        mmind::eye::profile_extraction::SpotSelection::name,
        static_cast<int>(mmind::eye::profile_extraction::SpotSelection::Value::Strongest)));

    // This parameter is only effective for firmware 2.2.1 and below. For firmware 2.3.0 and above,
    // adjustment of this parameter does not take effect.
    // Set the "Minimum Spot Intensity" parameter to 51
    showError(userSet.setIntValue(mmind::eye::profile_extraction::MinSpotIntensity::name, 51));
    // This parameter is only effective for firmware 2.2.1 and below. For firmware 2.3.0 and above,
    // adjustment of this parameter does not take effect.
    // Set the "Maximum Spot Intensity" parameter to 205
    showError(userSet.setIntValue(mmind::eye::profile_extraction::MaxSpotIntensity::name, 205));

    /* Set the "Gap Filling" parameter to 16, which controls the size of the gaps that can be filled
     * in the profile. When the number of consecutive data points in a gap in the profile is no
     * greater than 16, this gap will be filled. */
    showError(userSet.setIntValue(mmind::eye::profile_processing::GapFilling::name, 16));
    /* Set the "Filter" parameter to "Mean". The "Mean Filter Window Size" parameter needs to be set
     * as well. This parameter controls the window size of mean filter. If the "Filter" parameter is
     * set to "Median", the "Median Filter Window Size" parameter needs to be set. This parameter
     * controls the window size of median filter.*/
    showError(userSet.setEnumValue(
        mmind::eye::profile_processing::Filter::name,
        static_cast<int>(mmind::eye::profile_processing::Filter::Value::Mean)));
    // Set the "Mean Filter Window Size" parameter to 2
    showError(userSet.setEnumValue(
        mmind::eye::profile_processing::MeanFilterWindowSize::name,
        static_cast<int>(
            mmind::eye::profile_processing::MeanFilterWindowSize::Value::WindowSize_2)));
}

// Define the callback function for retrieving the profile data
void callbackFunc(const mmind::eye::ProfileBatch& batch, void* pUser)
{
    std::unique_lock<std::mutex> lock(kMutex);
    if (!batch.getErrorStatus().isOK()) {
        std::cout << "Error occurred during data acquisition." << std::endl;
        showError(batch.getErrorStatus());
    }
    auto* outPutBatch = static_cast<mmind::eye::ProfileBatch*>(pUser);
    outPutBatch->append(batch);
}

bool acquireProfileDataUsingCallback(mmind::eye::Profiler& profiler,
                                     mmind::eye::ProfileBatch& profileBatch, bool isSoftwareTrigger)
{
    profileBatch.clear();

    // Set a large value for CallbackRetrievalTimeout
    showError(profiler.currentUserSet().setIntValue(
        mmind::eye::scan_settings::CallbackRetrievalTimeout::name, 60000));

    // Register the callback function
    auto status = profiler.registerAcquisitionCallback(callbackFunc, &profileBatch);
    if (!status.isOK()) {
        showError(status);
        return false;
    }

    // Call startAcquisition() to enter the laser profiler into the acquisition ready status
    std::cout << "Start data acquisition." << std::endl;
    status = profiler.startAcquisition();
    if (!status.isOK()) {
        showError(status);
        return false;
    }

    // Call triggerSoftware() to start the data acquisition
    if (isSoftwareTrigger) {
        status = profiler.triggerSoftware();
        if (!status.isOK()) {
            showError(status);
            return false;
        }
    }

    while (true) {
        std::unique_lock<std::mutex> lock(kMutex);
        if (profileBatch.isEmpty()) {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else
            break;
    }

    std::cout << "Stop data acquisition." << std::endl;
    status = profiler.stopAcquisition();
    if (!status.isOK())
        showError(status);
    return status.isOK();
}

template <typename T, typename = std::enable_if_t<
                          (std::is_same<T, mmind::eye::ProfileBatch::PointCloud>::value ||
                           std::is_same<T, mmind::eye::ProfileBatch::TexturedPointCloud>::value)>>
bool containsInvalidPoint(const T& cloud)
{
    return std::any_of(
        cloud.data(), cloud.data() + cloud.width() * cloud.height() - 1, [](const auto& point) {
            return std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
                   std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z);
        });
}

pcl::PCLPointField createPointField(const std::string& name, uint32_t offset, uint8_t datatype,
                                    uint32_t count)
{
    pcl::PCLPointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = count;
    return field;
}

void convertToPCL(const mmind::eye::ProfileBatch::PointCloud& cloud,
                  pcl::PointCloud<pcl::PointXYZ>& pclPointCloud)
{
    pcl::PCLPointCloud2 pclCloud2;
    pclCloud2.height = cloud.height();
    pclCloud2.width = cloud.width();
    pclCloud2.point_step = sizeof(mmind::eye::PointXYZ);
    pclCloud2.row_step = sizeof(mmind::eye::PointXYZ) * cloud.width();
    pclCloud2.is_dense = !containsInvalidPoint<mmind::eye::ProfileBatch::PointCloud>(cloud);

    pclCloud2.fields.reserve(3);
    pclCloud2.fields.push_back(createPointField("x", offsetof(mmind::eye::PointXYZ, x),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("y", offsetof(mmind::eye::PointXYZ, y),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("z", offsetof(mmind::eye::PointXYZ, z),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));

    pclCloud2.data.resize(pclCloud2.row_step * cloud.height());
    memcpy(pclCloud2.data.data(),
           reinterpret_cast<uint8_t*>(const_cast<mmind::eye::PointXYZ*>(cloud.data())),
           (pclCloud2.row_step * cloud.height()));
    pcl::fromPCLPointCloud2(pclCloud2, pclPointCloud);
    return;
}

void convertToPCL(const mmind::eye::ProfileBatch::TexturedPointCloud& texturedCloud,
                  pcl::PointCloud<pcl::PointXYZI>& pclTexturedPointCloud)
{
    const auto height = texturedCloud.height();
    const auto width = texturedCloud.width();
    pclTexturedPointCloud.height = height;
    pclTexturedPointCloud.width = width;
    pclTexturedPointCloud.is_dense =
        !containsInvalidPoint<mmind::eye::ProfileBatch::TexturedPointCloud>(texturedCloud);
    pclTexturedPointCloud.points.resize(width * height);

    for (size_t row = 0; row < height; ++row) {
        for (size_t col = 0; col < width; ++col) {
            const mmind::eye::PointXYZI& src = texturedCloud.at(row, col);
            pcl::PointXYZI& dst = pclTexturedPointCloud.at(col, row);

            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            dst.intensity = src.intensity / 255.0f;
        }
    }

    return;
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

    // Acquire the profile data using the callback function
    if (!acquireProfileDataUsingCallback(profiler, profileBatch, isSoftwareTrigger))
        return -1;

    if (profileBatch.checkFlag(mmind::eye::ProfileBatch::BatchFlag::Incomplete))
        std::cout << "Part of the batch's data is lost, the number of valid profiles is: "
                  << profileBatch.validHeight() << "." << std::endl;

    const mmind::eye::ProfileBatch::PointCloud pointCloud =
        getUntexturedPointCloud(profileBatch, userSet);

    const std::string pointCloudFile = "UntexturedPointCloud.ply";
    pcl::PointCloud<pcl::PointXYZ> pointCloudPCL(pointCloud.width(), pointCloud.height());
    convertToPCL(pointCloud, pointCloudPCL);

    pcl::PLYWriter writer;
    writer.write(pointCloudFile, pointCloudPCL, true);
    std::cout << "The point cloud has: " << pointCloudPCL.width * pointCloudPCL.height
              << " data points." << std::endl;
    std::cout << "Save the untextured point cloud to file: " << pointCloudFile << std::endl;

    const mmind::eye::ProfileBatch::TexturedPointCloud texturedPointCloud =
        getTexturedPointCloud(profileBatch, userSet);

    std::string texturedPointCloudFile = "TexturedPointCloud.ply";
    pcl::PointCloud<pcl::PointXYZI> texturedPointCloudPCL(texturedPointCloud.width(),
                                                          texturedPointCloud.height());
    convertToPCL(texturedPointCloud, texturedPointCloudPCL);

    writer.write(texturedPointCloudFile, texturedPointCloudPCL, true);
    std::cout << "The point cloud has: "
              << texturedPointCloudPCL.width * texturedPointCloudPCL.height << " data points."
              << std::endl;
    std::cout << "Save the textured point cloud to file: " << texturedPointCloudFile << std::endl;

    profiler.disconnect();
    return 0;
}
