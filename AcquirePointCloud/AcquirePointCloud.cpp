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

#include <iostream>
#include <fstream>
#include <thread>
#include <cstdio>
#include "profiler/Profiler.h"
#include "profiler/api_util.h"
#include "profiler/parameters/scanParameters.h"
#include "profiler/parameters/ProfileExtractionParameters.h"

constexpr double kPitch = 1e-3;
constexpr int kInitEncoderValue = 0x0FFFFFFF;

int shiftEncoderValsAroundZero(unsigned int oriVal, int initValue = kInitEncoderValue)
{
    return static_cast<int>(oriVal - initValue);
}

void saveDataToCsv(float* data, int* encoderValues, int captureLineCount, int dataWidth,
                   float xUnit, int yUnit, const std::string& fileName)
{
    FILE* fp = fopen(fileName.data(), "w");
    if (fp) {
        std::cout << "Start to save point cloud!" << std::endl;
        for (int y = 0; y < captureLineCount; ++y) {
            for (int x = 0; x < dataWidth; ++x) {
                if (std::isnan(data[y * dataWidth + x]))
                    continue;
                fprintf(fp, "%f,%f,%f\n", static_cast<float>(x * xUnit * kPitch),
                        static_cast<float>(encoderValues[y] * yUnit * kPitch),
                        data[y * dataWidth + x]);
            }
        }
        fflush(fp);
        fclose(fp);
        std::cout << "Save point cloud to file: " << fileName << std::endl;
    } else {
        std::cout << "Fail to open file: " << fileName << std::endl;
    }
}

void capture(mmind::eye::Profiler& profiler, mmind::eye::ProfileBatch& totalBatch,
             std::vector<int>& encodersVals, int captureLineCount, int dataPoints)
{
    std::cout << "Start scanning!" << std::endl;

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
            totalBatch.append(batch);
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

    std::cout << "Please enter capture line count : ";
    int captureLineCnt;
    while (true) {
        std::string str;
        std::cin >> str;
        if (isNumber(str)) {
            captureLineCnt = atoi(str.c_str());
            break;
        }
        std::cout << "Input invalid! Please enter capture line count : ";
    }

    std::cout << "Please enter encoder trigger interval distance (unit : um): ";
    int yUnit;

    while (true) {
        std::string str;
        std::cin >> str;
        if (isNumber(str)) {
            yUnit = atoi(str.c_str());
            break;
        }
        std::cout << "Input invalid! Please enter encoder trigger interval distance (unit : um) "
                     "that must be integer : ";
    }

    if (!confirmCapture()) {
        profiler.disconnect();
        return -1;
    }

    mmind::eye::UserSet currentUserSet = profiler.currentUserSet();
    // Set the trigger source to Encoder
    showError(currentUserSet.setEnumValue(
        mmind::eye::trigger_settings::LineScanTriggerSource::name,
        static_cast<int>(mmind::eye::trigger_settings::LineScanTriggerSource::Value::Encoder)));

    // Set the maximum number of lines to be scanned to captureLineCnt
    showError(
        currentUserSet.setIntValue(mmind::eye::scan_settings::ScanLineCount::name, captureLineCnt));

    int dataPoints = 0;
    // Get the line width in the X direction
    showError(currentUserSet.getIntValue(mmind::eye::scan_settings::DataPointsPerProfile::name,
                                         dataPoints));
    int captureLineCount = 0;
    // Get the current maximum number of lines to be scanned
    currentUserSet.getIntValue(mmind::eye::scan_settings::ScanLineCount::name, captureLineCount);

    double xUnit = 0.0;
    showError(currentUserSet.getFloatValue(
        mmind::eye::point_cloud_resolutions::XAxisResolution::name, xUnit));

    std::string fileName = "PointCloud.csv";
    mmind::eye::ProfileBatch totalBatch(dataPoints);
    std::vector<int> encodersVals;
    encodersVals.reserve(captureLineCount);
    capture(profiler, totalBatch, encodersVals, captureLineCount, dataPoints);

    saveDataToCsv(totalBatch.getDepthMap().data(), encodersVals.data(), captureLineCount,
                  dataPoints, xUnit, yUnit, fileName);

    // Disconnect from theprofiler
    profiler.disconnect();
    return 0;
}
