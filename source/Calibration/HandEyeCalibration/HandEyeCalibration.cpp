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
With this sample program, you can perform hand-eye calibration and obtain the extrinsic parameters.
This document contains instructions for building the sample program and using the sample program to
complete hand-eye calibration.
*/

#include <opencv2/highgui/highgui.hpp>
#include "HandEyeCalibrationUtil.h"
#include "OpenCVUtil.h"
int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    std::cout << "Connected to the Mech-Eye device successfully." << std::endl;

    // Set the model of the calibration board.
    mmind::api::HandeyeCalibrationSettings::boardType boardType = inputBoardType();
    while (boardType == mmind::api::HandeyeCalibrationSettings::boardType::UNKNOWN) {
        std::cout << "Please enter again." << std::endl;
        boardType = inputBoardType();
    }
    showError(device.setBoardType(boardType));

    // Set the camera mounting method.
    mmind::api::HandeyeCalibrationSettings::calibrationType calibrateType = inputCalibType();
    while (calibrateType == mmind::api::HandeyeCalibrationSettings::calibrationType::UNKNOWN) {
        std::cout << "Please enter again." << std::endl;
        calibrateType = inputCalibType();
    }
    showError(device.setCalibrateType(calibrateType));

    // Set the Euler angle convention.
    auto eulerType = getEulerType();
    while (eulerType == 0) // Prompt to enter again if the entered number is invalid.
    {
        std::cout << "Invalid Euler angle convention. Please enter again." << std::endl;
        eulerType = getEulerType();
    }

    std::cout << "\n******************************************************************************"
                 "\nExtrinsic parameter calculation requires at least 15 robot poses."
                 "\nDuring the hand-eye calibration, please make sure you enter enough robot poses"
                 "\nat which the feature detection of the 2D image is successful."
                 "\n******************************************************************************"
              << std::endl;
    int poseIndex = 1;
    std::string extrinsic;
    bool calibrate{false};
    do {
        switch (enterCommand()) {
        case CommandType::GetOriginImg: // Obtain the original 2D image.
        {
            mmind::api::ColorMap color;
            showError(device.captureColorMap(color));
            if (!color.empty()) {
                std::string colorFile = "Original2DImage" + std::to_string(poseIndex);
                colorFile += ".png";
                cv::Mat testImg = cv::Mat(color.height(), color.width(), CV_8UC3, color.data());
                cv::namedWindow("Original 2D Image", 0);
                cv::resizeWindow("Original 2D Image", color.width() / 2, color.height() / 2);
                cv::imshow("Original 2D Image", testImg);
                cv::waitKey(0);
                cv::destroyAllWindows();
                saveMap(color, colorFile);
            }
            break;
        }
        case CommandType::GetPatternImg: // Obtain the 2D image with feature point recognition
                                         // results.
        {
            mmind::api::ColorMap color;
            showError(device.captureCalibrationFeatureImage(color));
            if (!color.empty()) {
                std::string colorFile = "FeatureRecognitionResult" + std::to_string(poseIndex);
                colorFile += ".png";
                cv::Mat testImg = cv::Mat(color.height(), color.width(), CV_8UC3, color.data());
                cv::namedWindow("Feature Recognition Result", 0);
                cv::resizeWindow("Feature Recognition Result", color.width() / 2,
                                 color.height() / 2);
                cv::imshow("Feature Recognition Result", testImg);
                cv::waitKey(0);
                cv::destroyAllWindows();
                saveMap(color, colorFile);
            }
            break;
        }
        case CommandType::AddPose: // Input the current robot pose. The unit of the translational
                                   // components is mm, and the unit of the Euler angles is degrees.

        {
            auto robotPose = enterRobotPose(eulerType);
            std::cout << "\nThe current pose index is " << poseIndex
                      << "\nIf the above poses are correct, enter y; otherwise, press any key to "
                         "enter the pose again."
                      << std::endl;
            std::string key;
            std::cin >> key;
            while (key != "y") {
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << std::endl;
                std::cout << "Enter the pose again:" << std::endl;
                robotPose = enterRobotPose(eulerType);
                std::cout
                    << "\nThe current pose index is " << poseIndex
                    << "\nIf the above poses are correct, enter y; otherwise, press any key to "
                       "re-enter the pose."
                    << std::endl;
                std::cin >> key;
            }
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            auto errStatus = device.addPoseAndDetect(robotPose);
            showError(errStatus);
            if (errStatus.isOK()) {
                poseIndex++;
            }
            break;
        }
        case CommandType::Calibrate: // Calculate extrinsic parameters.
        {
            calibrate = true;
            showError(device.calculateExtrinsics(extrinsic));
            if (!extrinsic.empty()) {
                std::cout << "Extrinsic is" << std::endl;
                std::cout << extrinsic << std::endl;
                saveExtrinsicParameters(extrinsic);
            }
            break;
        }
        case CommandType::Unknown:
        {
            std::cout << "Error: Unknown command" << std::endl;
            break;
        }
        }
    } while (!calibrate);
    device.disconnect();
    std::cout << "Disconnected from the Mech-Eye device successfully." << std::endl;
    return 0;
}
