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
With this sample, you can set the parameters in the "3D Parameters", "2D Parameters", and "ROI"
categories.
*/
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/api_util.h"
#include "area_scan_3d_camera/parameters/Scanning3D.h"
#include "area_scan_3d_camera/parameters/Scanning2D.h"

int main()
{
    // List all available cameras and connect to a camera by the displayed index.
    mmind::eye::Camera camera;
    if (!findAndConnect(camera))
        return -1;

    // Obtain the basic information of the connected camera.
    mmind::eye::CameraInfo cameraInfo;
    showError(camera.getCameraInfo(cameraInfo));
    printCameraInfo(cameraInfo);

    mmind::eye::UserSet& currentUserSet = camera.currentUserSet();

    // Set the exposure times for acquiring depth information.
    showError(currentUserSet.setFloatArrayValue(
        mmind::eye::scanning3d_setting::ExposureSequence::name, std::vector<double>{5}));
    //    showError(currentUserSet.setFloatArrayValue(
    //        mmind::eye::scanning3d_setting::ExposureSequence::name, std::vector<double>{5, 10}));

    // Obtain the current exposure times for acquiring depth information to check if the setting was
    // successful.
    std::vector<double> exposureSequence;
    showError(currentUserSet.getFloatArrayValue(
        mmind::eye::scanning3d_setting::ExposureSequence::name, exposureSequence));
    std::cout << "3D scanning exposure multiplier : " << exposureSequence.size() << "."
              << std::endl;
    for (size_t i = 0; i < exposureSequence.size(); i++) {
        std::cout << "3D scanning exposure time " << i + 1 << ": " << exposureSequence[i] << " ms."
                  << std::endl;
    }

    // Set the ROI for the depth map and point cloud, and then obtain the parameter value to check
    // if the setting was successful.
    mmind::eye::ROI roi(0, 0, 500, 500);
    showError(currentUserSet.setRoiValue(mmind::eye::scanning3d_setting::ROI::name, roi));
    mmind::eye::ROI curRoi;
    showError(currentUserSet.getRoiValue(mmind::eye::scanning3d_setting::ROI::name, curRoi));
    std::cout << "3D scanning ROI topLeftX: " << curRoi.upperLeftX
              << ", topLeftY: " << curRoi.upperLeftY << ", width: " << curRoi.width
              << ", height: " << curRoi.height << std::endl;

    // Set the exposure mode and exposure time for acquiring the 2D image, and then obtain the
    // parameter values to check if the setting was successful.
    showError(currentUserSet.setEnumValue(
        mmind::eye::scanning2d_setting::ExposureMode::name,
        static_cast<int>(mmind::eye::scanning2d_setting::ExposureMode::Value::Timed)));

    showError(
        currentUserSet.setFloatValue(mmind::eye::scanning2d_setting::ExposureTime::name, 100));

    // The DEEP and LSR series also provide a "Scan2DPatternRoleExposureMode" parameter for
    // adjusting the exposure mode for acquiring the 2D images (depth source). Uncomment the
    // following lines to set this parameter to "Timed".
    // showError(currentUserSet.setEnumValue(
    //     mmind::eye::scanning2d_setting::ExposureMode::name,
    //     static_cast<int>(mmind::eye::scanning2d_setting::DepthSourceExposureMode::Value::Timed)));

    // You can also use the projector for supplemental light when acquiring the 2D image / 2D images
    // (depth source).
    // Models other than the DEEP and LSR series: Uncomment the following lines to set the exposure
    // mode to "Flash" for supplemental light.
    //     showError(currentUserSet.setEnumValue(
    //         mmind::eye::scanning2d_setting::ExposureMode::name,
    //         static_cast<int>(mmind::eye::scanning2d_setting::ExposureMode::Value::Flash)));
    // DEEP and LSR series: Uncomment the following lines to set the exposure mode to "Flash" for
    // supplemental light.
    // showError(currentUserSet.setEnumValue(
    //     mmind::eye::scanning2d_setting::ExposureMode::name,
    //     static_cast<int>(mmind::eye::scanning2d_setting::DepthSourceExposureMode::Value::Flash)));

    // The following models also provide a "FlashAcquisitionMode" when using the flash exposure
    // mode: DEEP, DEEP-GL, LSR S/L/XL, LSR S-GL/L-GL/XL-GL, PRO XS/S/M, PRO XS-GL/S-GL/M-GL, NANO,
    // NANO-GL, NANO ULTRA, NANO ULTRA-GL. Uncomment the following lines to set the
    // "FlashAcquisitionMode" parameter to "Responsive".
    //  showError(currentUserSet.setEnumValue(
    //      mmind::eye::scanning2d_setting::FlashAcquisitionMode::name,
    //      static_cast<int>(mmind::eye::scanning2d_setting::FlashAcquisitionMode::Value::Responsive)));

    // When using the responsive acquisition mode, you can adjust the exposure time for the flash
    // exposure mode. Uncomment the following lines to set the exposure time to 20 ms.
    //  showError(
    //     currentUserSet.setFloatValue(mmind::eye::scanning2d_setting::FlashExposureTime::name,
    //     20));

    // Uncomment the following lines to check the values of the "FlashAcquisitionMode" and
    // "FlashExposureTime" parameters. int flashAcquisitionMode = 0;
    // showError(currentUserSet.getEnumValue(
    //     mmind::eye::scanning2d_setting::FlashAcquisitionMode::name, flashAcquisitionMode));

    // double flashExposureTime = 0;
    // showError(currentUserSet.getFloatValue(mmind::eye::scanning2d_setting::FlashExposureTime::name,
    //                                        flashExposureTime));

    // std::cout << "2D scanning flash acquisition mode: " << flashAcquisitionMode
    //           << ", flash exposure time: " << flashExposureTime << " ms." << std::endl;

    // Uncomment the following line to set the camera gain when capturing 2D images.

    // For DEEP and LSR series cameras, the camera gain for capturing 2D images (texture) can be
    // adjusted by setting the "Scan2DGain" parameter.

    // For other camera series, the camera gain for capturing 2D images can be adjusted by
    // setting the "Scan2DGain" parameter when the exposure mode is set to fixed exposure, auto
    // exposure, HDR, or flash mode, and the flash acquisition mode is set to
    // responsive.

    // showError(currentUserSet.setFloatValue(mmind::eye::scanning2d_setting::Gain::name, 2.0));

    // double scan2dGain{};
    // showError(currentUserSet.getFloatValue(mmind::eye::scanning2d_setting::Gain::name,
    // scan2dGain)); std::cout << "2D image gain: " << scan2dGain << " dB." << std::endl;

    int exposureMode2D = 0;
    showError(currentUserSet.getEnumValue(mmind::eye::scanning2d_setting::ExposureMode::name,
                                          exposureMode2D));
    double scan2DExposureTime = 0;
    showError(currentUserSet.getFloatValue(mmind::eye::scanning2d_setting::ExposureTime::name,
                                           scan2DExposureTime));
    std::cout << "2D scanning exposure mode: " << exposureMode2D
              << ", exposure time: " << scan2DExposureTime << " ms." << std::endl;

    // Save all the parameter settings to the currently selected user sets.
    std::string successMessage = "Save all parameters to the current user set.";
    showError(currentUserSet.saveAllParametersToDevice(), successMessage);

    camera.disconnect();
    std::cout << "Disconnected from the camera successfully." << std::endl;
    return 0;
}
