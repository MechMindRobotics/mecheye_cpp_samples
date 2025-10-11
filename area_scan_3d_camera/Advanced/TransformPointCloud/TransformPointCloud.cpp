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
With this sample, you can retrieve point clouds in the custom reference frame.
*/

#include <area_scan_3d_camera/Camera.h>
#include <area_scan_3d_camera/api_util.h>
#include <area_scan_3d_camera/parameters/Scanning3D.h>
#include <CommonTypes.h>
#include <vector>
#include "area_scan_3d_camera/PointCloudTransformation.h"

int main()
{
    mmind::eye::Camera camera;
    if (!findAndConnect(camera))
        return 0;

    mmind::eye::CameraInfo cameraInfo;
    showError(camera.getCameraInfo(cameraInfo));
    printCameraInfo(cameraInfo);

    /*
     * Obtain the rigid body transformation from the camera reference frame to the custom reference
     * frame.
     * The custom reference frame can be adjusted using the "Custom Reference Frame" tool in
     * Mech-Eye Viewer. The rigid body transformations are automatically calculated. Alternatively,
     * you can avoid using the current interface and instead use the rotation and translation
     * methods of FrameTransformation to construct a transformation matrix manually. However, this
     * method is not recommended because it is less intuitive.
     */
    mmind::eye::FrameTransformation transformation = getTransformationParams(camera);
    if (!transformation.isValid()) {
        std::cout << "Transformation parameters are not set. Please configure the transformation "
                     "parameters using the custom coordinate system tool in the client."
                  << std::endl;
    }

    // Get the untextured point cloud
    mmind::eye::Frame3D frame3D;
    showError(camera.capture3D(frame3D));
    if (frame3D.isEmpty()) {
        std::cout << "The obtained data is empty. The program is terminated." << std::endl;
        return 0;
    }

    // Transform the reference frame of the untextured point cloud and untextured point cloud with
    // normals, and save the point cloud
    mmind::eye::PointCloud transformedPointCloud =
        mmind::eye::transformPointCloud(transformation, frame3D.getUntexturedPointCloud());
    const std::string pointCloudFile = "PointCloud.ply";

    std::string successMessage = "Save the point cloud:" + pointCloudFile;
    showError(
        frame3D.savePointCloud(transformedPointCloud, mmind::eye::FileFormat::PLY, pointCloudFile),
        successMessage);

    mmind::eye::PointCloudWithNormals transformedPointCloudWithNormals =
        mmind::eye::transformPointCloudWithNormals(transformation,
                                                   frame3D.getUntexturedPointCloud());
    const std::string pointCloudWithNormalsFile = "PointCloudWithNormals.ply";
    successMessage = "Save the point cloud with normals:" + pointCloudWithNormalsFile;
    showError(
        frame3D.savePointCloudWithNormals(transformedPointCloudWithNormals,
                                          mmind::eye::FileFormat::PLY, pointCloudWithNormalsFile),
        successMessage);

    // Get the textured point cloud
    mmind::eye::Frame2DAnd3D frame2DAnd3D;
    showError(camera.capture2DAnd3D(frame2DAnd3D));
    if (frame2DAnd3D.frame2D().isEmpty() || frame2DAnd3D.frame3D().isEmpty()) {
        std::cout << "The obtained data is empty. The program is terminated." << std::endl;
        return 0;
    }

    // Transform the reference frame of the textured point cloud and textured point cloud with
    // normals, and save the point cloud
    mmind::eye::TexturedPointCloud transformedTexturedPointCloud =
        mmind::eye::transformTexturedPointCloud(transformation,
                                                frame2DAnd3D.getTexturedPointCloud());
    const std::string texturedPointCloudFile = "TexturedPointCloud.ply";
    successMessage = "Capture and save the textured point cloud:" + texturedPointCloudFile;
    showError(frame2DAnd3D.savePointCloud(transformedTexturedPointCloud,
                                          mmind::eye::FileFormat::PLY, texturedPointCloudFile),
              successMessage);

    mmind::eye::TexturedPointCloudWithNormals transformedTexturedPointCloudWithNormals =
        mmind::eye::transformTexturedPointCloudWithNormals(transformation,
                                                           frame2DAnd3D.getTexturedPointCloud());
    const std::string texturedPointCloudWithNormalsFile = "TexturedPointCloudWithNormals.ply";
    successMessage = "Capture and save the textured point cloud with normals:" +
                     texturedPointCloudWithNormalsFile;
    showError(frame2DAnd3D.savePointCloudWithNormals(transformedTexturedPointCloudWithNormals,
                                                     mmind::eye::FileFormat::PLY,
                                                     texturedPointCloudWithNormalsFile),
              successMessage);

    camera.disconnect();
    std::cout << "Disconnected from the camera successfully." << std::endl;
    return 0;
}
