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
With this sample, you can obtain the point cloud data with normals from the camera and convert it to
the PCL data structure.
*/

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkOutputWindow.h>
#include <string>
#include <thread>
#include <cstddef>
#include "area_scan_3d_camera/api_util.h"
#include "area_scan_3d_camera/Camera.h"

namespace {

bool containsInvalidPoint(const mmind::eye::PointCloudWithNormals& cloud)
{
    return std::any_of(
        cloud.data(), cloud.data() + cloud.width() * cloud.height() - 1,
        [](const auto& pointWithNormals) {
            return std::isnan(pointWithNormals.point.x) || std::isnan(pointWithNormals.point.y) ||
                   std::isnan(pointWithNormals.point.z) || std::isnan(pointWithNormals.normal.x) ||
                   std::isnan(pointWithNormals.normal.y) || std::isnan(pointWithNormals.normal.z) ||
                   std::isinf(pointWithNormals.point.x) || std::isinf(pointWithNormals.point.y) ||
                   std::isinf(pointWithNormals.point.z) || std::isinf(pointWithNormals.normal.x) ||
                   std::isinf(pointWithNormals.normal.y) || std::isinf(pointWithNormals.normal.z);
        });
}

bool containsInvalidPoint(const mmind::eye::TexturedPointCloudWithNormals& cloud)
{
    return std::any_of(cloud.data(), cloud.data() + cloud.width() * cloud.height() - 1,
                       [](const auto& pointWithNormals) {
                           return std::isnan(pointWithNormals.colorPoint.x) ||
                                  std::isnan(pointWithNormals.colorPoint.y) ||
                                  std::isnan(pointWithNormals.colorPoint.z) ||
                                  std::isnan(pointWithNormals.normal.x) ||
                                  std::isnan(pointWithNormals.normal.y) ||
                                  std::isnan(pointWithNormals.normal.z) ||
                                  std::isinf(pointWithNormals.colorPoint.x) ||
                                  std::isinf(pointWithNormals.colorPoint.y) ||
                                  std::isinf(pointWithNormals.colorPoint.z) ||
                                  std::isinf(pointWithNormals.normal.x) ||
                                  std::isinf(pointWithNormals.normal.y) ||
                                  std::isinf(pointWithNormals.normal.z);
                       });
}

<<<<<<< HEAD
pcl::PCLPointField createPointField(std::string name, uint32_t offset, uint8_t datatype,
                                    uint32_t count)
{
    pcl::PCLPointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = count;
    return field;
}

void convertToPCL(const mmind::eye::PointCloudWithNormals& cloud,
                  pcl::PointCloud<pcl::PointNormal>& pclPointCloud)
{
    pcl::PCLPointCloud2 pclCloud2;
    pclCloud2.height = cloud.height();
    pclCloud2.width = cloud.width();
    pclCloud2.point_step = sizeof(mmind::eye::PointXYZWithNormals);
    pclCloud2.row_step = sizeof(mmind::eye::PointXYZWithNormals) * cloud.width();
    pclCloud2.is_dense = !containsInvalidPoint(cloud);
=======
void convertToPCL(const mmind::eye::PointCloudWithNormals& cloud,
                  pcl::PointCloud<pcl::PointNormal>& pclPointCloud)
{
    // write PointNormal data
    uint32_t size = cloud.height() * cloud.width();
    pclPointCloud.resize(size);
    pclPointCloud.is_dense = !containsInvalidPoint(cloud);
>>>>>>> bd8af32f5562b00c36cf7d083d9db94308a7b831

    pclCloud2.fields.reserve(7);
    pclCloud2.fields.push_back(createPointField("x",
                                                offsetof(mmind::eye::PointXYZWithNormals, point.x),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("y",
                                                offsetof(mmind::eye::PointXYZWithNormals, point.y),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("z",
                                                offsetof(mmind::eye::PointXYZWithNormals, point.z),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("normal_x",
                                                offsetof(mmind::eye::PointXYZWithNormals, normal.x),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("normal_y",
                                                offsetof(mmind::eye::PointXYZWithNormals, normal.y),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("normal_z",
                                                offsetof(mmind::eye::PointXYZWithNormals, normal.z),
                                                pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(
        createPointField("curvature", offsetof(mmind::eye::PointXYZWithNormals, normal.curvature),
                         pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));

    pclCloud2.data.resize(pclCloud2.row_step * cloud.height());
    memcpy(pclCloud2.data.data(),
           reinterpret_cast<uint8_t*>(const_cast<mmind::eye::PointXYZWithNormals*>(cloud.data())),
           (pclCloud2.row_step * cloud.height()));

    pcl::fromPCLPointCloud2(pclCloud2, pclPointCloud);
}

void convertToPCL(const mmind::eye::TexturedPointCloudWithNormals& texturedCloud,
                  pcl::PointCloud<pcl::PointXYZRGBNormal>& pclTexturedPointCloud)
{
    // write PointXYZRGBNormal data
<<<<<<< HEAD
    pcl::PCLPointCloud2 pclCloud2;
    pclCloud2.height = texturedCloud.height();
    pclCloud2.width = texturedCloud.width();
    pclCloud2.point_step = sizeof(mmind::eye::PointXYZBGRWithNormals);
    pclCloud2.row_step = sizeof(mmind::eye::PointXYZBGRWithNormals) * texturedCloud.width();
    pclCloud2.is_dense = !containsInvalidPoint(texturedCloud);
=======
    uint32_t size = texturedCloud.height() * texturedCloud.width();
    pclTexturedPointCloud.resize(size);
    pclTexturedPointCloud.is_dense =
        !containsInvalidPoint(texturedCloud);
>>>>>>> bd8af32f5562b00c36cf7d083d9db94308a7b831

    pclCloud2.fields.reserve(8);
    pclCloud2.fields.push_back(
        createPointField("x", offsetof(mmind::eye::PointXYZBGRWithNormals, colorPoint.x),
                         pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(
        createPointField("y", offsetof(mmind::eye::PointXYZBGRWithNormals, colorPoint.y),
                         pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(
        createPointField("z", offsetof(mmind::eye::PointXYZBGRWithNormals, colorPoint.z),
                         pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(
        createPointField("rgb", offsetof(mmind::eye::PointXYZBGRWithNormals, colorPoint.rgb),
                         pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(
        createPointField("normal_x", offsetof(mmind::eye::PointXYZBGRWithNormals, normal.x),
                         pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(
        createPointField("normal_y", offsetof(mmind::eye::PointXYZBGRWithNormals, normal.y),
                         pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(
        createPointField("normal_z", offsetof(mmind::eye::PointXYZBGRWithNormals, normal.z),
                         pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField(
        "curvature", offsetof(mmind::eye::PointXYZBGRWithNormals, normal.curvature),
        pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));

    pclCloud2.data.resize(pclCloud2.row_step * texturedCloud.height());
    std::memcpy(pclCloud2.data.data(),
                reinterpret_cast<uint8_t*>(
                    const_cast<mmind::eye::PointXYZBGRWithNormals*>(texturedCloud.data())),
                (pclCloud2.row_step * texturedCloud.height()));

    pcl::fromPCLPointCloud2(pclCloud2, pclTexturedPointCloud);
}

void showPointCloud(const pcl::PointCloud<pcl::PointNormal>& pointCloud)
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    if (pointCloud.empty())
        return;

    pcl::visualization::PCLVisualizer cloudViewer("Point Cloud Viewer");
    cloudViewer.setShowFPS(false);
    cloudViewer.setBackgroundColor(0, 0, 0);
    cloudViewer.addPointCloudNormals<pcl::PointNormal>(pointCloud.makeShared());
    cloudViewer.addCoordinateSystem(0.01);
    cloudViewer.addText("Point cloud size: " + std::to_string(pointCloud.size()), 0, 25, 20, 1, 1,
                        1, "cloudSize");
    cloudViewer.addText("Press r/R to reset camera view point to center.", 0, 0, 16, 1, 1, 1,
                        "help");
    cloudViewer.initCameraParameters();
    while (!cloudViewer.wasStopped()) {
        cloudViewer.spinOnce(20);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void showPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>& colorPointCloud)
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    if (colorPointCloud.empty())
        return;

    pcl::visualization::PCLVisualizer cloudViewer("Point Cloud Viewer");
    cloudViewer.setShowFPS(false);
    cloudViewer.setBackgroundColor(0, 0, 0);
    cloudViewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(colorPointCloud.makeShared());
    cloudViewer.addCoordinateSystem(0.01);
    cloudViewer.addText("Point cloud size: " + std::to_string(colorPointCloud.size()), 0, 25, 20, 1,
                        1, 1, "cloudSize");
    cloudViewer.addText("Press r/R to reset camera view point to center.", 0, 0, 16, 1, 1, 1,
                        "help");
    cloudViewer.initCameraParameters();
    while (!cloudViewer.wasStopped()) {
        cloudViewer.spinOnce(20);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

} // namespace

int main()
{
    mmind::eye::Camera camera;
    if (!findAndConnect(camera))
        return -1;

    if (!confirmCapture3D()) {
        camera.disconnect();
        return 0;
    }

    camera.setPointCloudUnit(mmind::eye::CoordinateUnit::Meter);
    mmind::eye::Frame2DAnd3D frame2DAnd3D;
    showError(camera.capture2DAnd3DWithNormal(frame2DAnd3D));

    const mmind::eye::PointCloudWithNormals pointCloud =
        frame2DAnd3D.frame3D().getUntexturedPointCloudWithNormals();

    const std::string pointCloudFile = "UntexturedPointCloudWithNormals.ply";
    pcl::PointCloud<pcl::PointNormal> pointCloudPCL(pointCloud.width(), pointCloud.height());
    convertToPCL(pointCloud, pointCloudPCL);

    showPointCloud(pointCloudPCL);

    pcl::PLYWriter writer;
    writer.write(pointCloudFile, pointCloudPCL, true);
    std::cout << "The point cloud has: " << pointCloudPCL.width * pointCloudPCL.height
              << " data points." << std::endl;
    std::cout << "Save the untextured point cloud with normals to file:" << pointCloudFile
              << std::endl;

    const mmind::eye::TexturedPointCloudWithNormals texturedPointCloud =
        frame2DAnd3D.getTexturedPointCloudWithNormals();

    std::string texturedPointCloudFile = "TexturedPointCloudWithNormals.ply";
    pcl::PointCloud<pcl::PointXYZRGBNormal> texturedPointCloudPCL(texturedPointCloud.width(),
                                                                  texturedPointCloud.height());
    convertToPCL(texturedPointCloud, texturedPointCloudPCL);

    showPointCloud(texturedPointCloudPCL);
    writer.write(texturedPointCloudFile, texturedPointCloudPCL, true);
    std::cout << "The point cloud has: "
              << texturedPointCloudPCL.width * texturedPointCloudPCL.height << " data points."
              << std::endl;
    std::cout << "Save the textured point cloud with normals to file: " << texturedPointCloudFile
              << std::endl;

    camera.disconnect();
    std::cout << "Disconnected from the camera successfully." << std::endl;

    return 0;
}
