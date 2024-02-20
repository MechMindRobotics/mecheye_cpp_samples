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
With this sample, you can acquire the profile data, generate the point cloud, and save the point
cloud in the CSV and PLY formats
*/

#include <iostream>
#include <fstream>
#include <thread>
#include <regex>
#include <cstdio>
#include <cmath>
#include "profiler/Profiler.h"
#include "profiler/api_util.h"
#include "profiler/parameters/ScanParameters.h"
#include "profiler/parameters/ProfileExtractionParameters.h"

constexpr double kPitch = 1e-3;
constexpr long long kInitEncoderValue = 0x0FFFFFFF;

int shiftEncoderValsAroundZero(unsigned int oriVal, long long initValue = kInitEncoderValue)
{
    return static_cast<int>(oriVal - initValue);
}

bool saveDataToPly(float* data, int* encoderValues, int captureLineCount, int dataWidth,
                   float xUnit, float yUnit, const std::string& fileName, bool isOrganized,
                   bool useEncoderValues = true)
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

    for (int y = 0; y < captureLineCount; ++y) {
        for (int x = 0; x < dataWidth; ++x) {
            if (!std::isnan(data[y * dataWidth + x]))
                fprintf(
                    fp, "%f %f %f\n", static_cast<float>(x * xUnit * kPitch),
                    static_cast<float>((useEncoderValues ? encoderValues[y] : y) * yUnit * kPitch),
                    data[y * dataWidth + x]);
            else if (isOrganized)
                fprintf(fp, "nan nan nan\n");
        }
    }

    fclose(fp);
    return true;
}

bool saveDataToCsv(float* data, int* encoderValues, int captureLineCount, int dataWidth,
                   float xUnit, float yUnit, const std::string& fileName, bool isOrganized,
                   bool useEncoderValues = true)
{
    FILE* fp = fopen(fileName.c_str(), "w");

    if (!fp)
        return false;

    fprintf(fp, "X,Y,Z\n");

    for (int y = 0; y < captureLineCount; ++y) {
        for (int x = 0; x < dataWidth; ++x) {
            if (!std::isnan(data[y * dataWidth + x]))
                fprintf(
                    fp, "%f,%f,%f\n", static_cast<float>(x * xUnit * kPitch),
                    static_cast<float>((useEncoderValues ? encoderValues[y] : y) * yUnit * kPitch),
                    data[y * dataWidth + x]);
            else if (isOrganized)
                fprintf(fp, "nan,nan,nan\n");
        }
    }

    fclose(fp);
    return true;
}

void capture(mmind::eye::Profiler& profiler, mmind::eye::ProfileBatch& totalBatch,
             std::vector<int>& encodersVals, int captureLineCount, int dataPoints)
{
    std::cout << "Start data acquisition." << std::endl;

    auto status = profiler.startAcquisition();
    if (!status.isOK())
        return;

    status = profiler.triggerSoftware();
    if (!status.isOK())
        return;

    totalBatch.reserve(captureLineCount);
    while (totalBatch.height() < captureLineCount) {
        mmind::eye::ProfileBatch batch(dataPoints);
        mmind::eye::ErrorStatus status = profiler.retrieveBatchData(batch);

        if (status.isOK()) {
            if (!totalBatch.append(batch))
                break;
            for (int r = 0; r < batch.height(); ++r) {
                mmind::eye::ProfileBatch::EncoderArray encoder = totalBatch.getEncoderArray();
                encodersVals.push_back(shiftEncoderValsAroundZero(
                    encoder[totalBatch.height() - batch.height() + r], encoder[0]));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } else {
            showError(status);
            break;
        }
    }
}

int main()
{
    mmind::eye::Profiler profiler;
    if (!findAndConnect(profiler))
        return -1;

    if (!confirmCapture()) {
        profiler.disconnect();
        return -1;
    }

    std::cout << "Please enter the number of lines that you want to scan (min: 16, max: 60000): ";
    int captureLineCnt;
    while (true) {
        std::string str;
        std::cin >> str;
        if (std::regex_match(str.begin(), str.end(), std::regex{"[0-9]+"})) {
            captureLineCnt = atoi(str.c_str());
            break;
        }
        std::cout << "Input invalid! Please enter the number of lines that you want to scan (min: "
                     "16, max: 60000): ";
    }

    mmind::eye::UserSet currentUserSet = profiler.currentUserSet();

    // Set the "Data Acquisition Trigger Source" parameter to "Software"
    showError(currentUserSet.setEnumValue(
        mmind::eye::trigger_settings::DataAcquisitionTriggerSource::name,
        static_cast<int>(
            mmind::eye::trigger_settings::DataAcquisitionTriggerSource::Value::Software)));
    //// Set the "Data Acquisition Trigger Source" parameter to "External"
    // showError(currentUserSet.setEnumValue(
    //     mmind::eye::trigger_settings::DataAcquisitionTriggerSource::name,
    //     static_cast<int>(mmind::eye::trigger_settings::DataAcquisitionTriggerSource::Value::External)));

    // Set the "Line Scan Trigger Source" parameter to "Encoder"
    showError(currentUserSet.setEnumValue(
        mmind::eye::trigger_settings::LineScanTriggerSource::name,
        static_cast<int>(mmind::eye::trigger_settings::LineScanTriggerSource::Value::Encoder)));
    // // Set the "Line Scan Trigger Source" parameter to "FixedRate"
    // showError(currentUserSet.setEnumValue(
    // mmind::eye::trigger_settings::LineScanTriggerSource::name,
    // static_cast<int>(mmind::eye::trigger_settings::LineScanTriggerSource::Value::FixedRate)));

    // Set the "Scan Line Count" parameter (the number of lines to be scanned) to captureLineCnt
    showError(
        currentUserSet.setIntValue(mmind::eye::scan_settings::ScanLineCount::name, captureLineCnt));

    int dataPoints = 0;
    // Get the number of data points in each profile
    showError(currentUserSet.getIntValue(mmind::eye::scan_settings::DataPointsPerProfile::name,
                                         dataPoints));
    int captureLineCount = 0;
    // Get the current value of the "Scan Line Count" parameter
    currentUserSet.getIntValue(mmind::eye::scan_settings::ScanLineCount::name, captureLineCount);

    // Get the X-axis resolution
    double xUnit = 0.0;
    showError(currentUserSet.getFloatValue(
        mmind::eye::point_cloud_resolutions::XAxisResolution::name, xUnit));

    // Ge the Y resolution
    double yUnit;
    showError(currentUserSet.getFloatValue(mmind::eye::point_cloud_resolutions::YResolution::name,
                                           yUnit));
    // Uncomment the following lines for custom Y Unit
    // // Prompt to enter the desired encoder resolution, which is the travel distance corresponding
    // to
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

    mmind::eye::ProfileBatch totalBatch(dataPoints);
    std::vector<int> encodersVals;
    encodersVals.reserve(captureLineCount);
    capture(profiler, totalBatch, encodersVals, captureLineCount, dataPoints);

    if (!totalBatch.isEmpty()) {
        saveDataToCsv(totalBatch.getDepthMap().data(), encodersVals.data(), captureLineCount,
                      dataPoints, xUnit, yUnit, "PointCloud.csv", true);
        saveDataToPly(totalBatch.getDepthMap().data(), encodersVals.data(), captureLineCount,
                      dataPoints, xUnit, yUnit, "PointCloud.ply", true);

        // Comment the lines above and uncomment the lines below if you set "Line Scan Trigger
        // Source" to "FixedRate". Source" to "FixedRate".
        // saveDataToCsv(totalBatch.getDepthMap().data(), encodersVals.data(), captureLineCount,
        //   dataPoints, xUnit, yUnit, "PointCloud.csv", true, false);
        // saveDataToPly(totalBatch.getDepthMap().data(), encodersVals.data(), captureLineCount,
        //               dataPoints, xUnit, yUnit, "PointCloud.ply", true, false);
    }

    // Disconnect from the laser profiler
    profiler.disconnect();
    return 0;
}
