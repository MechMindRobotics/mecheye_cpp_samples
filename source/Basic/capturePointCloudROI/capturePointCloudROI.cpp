#include <iostream>
#include <thread>

#include <vtkOutputWindow.h>

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include "PclUtil.h"

int main()
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    // Set 3D ROI
    showError(device.setScan3DROI(mmind::api::ROI(0, 0, 500, 500)));

    mmind::api::PointXYZMap pointXYZMap;
    device.capturePointXYZMap(pointXYZMap);

    mmind::api::PointXYZBGRMap pointXYZBGRMap;
    device.capturePointXYZBGRMap(pointXYZBGRMap);

    std::string pointCloudPath = "pointCloudXYZ.ply";
    pcl::PointCloud<pcl::PointXYZ> pointCloud(pointXYZMap.width(), pointXYZMap.height());
    toPCL(pointCloud, pointXYZMap);
    viewPCL(pointCloud);
    pcl::PLYWriter writer;
    writer.write(pointCloudPath, pointCloud, true);
    std::cout << "PointCloud has : " << pointCloud.width * pointCloud.height << " data points."
              << std::endl;

    std::string colorPointCloudPath = "pointCloudXYZRGB.ply";
    pcl::PointCloud<pcl::PointXYZRGB> colorPointCloud(pointXYZBGRMap.width(),
                                                      pointXYZBGRMap.height());
    toPCL(colorPointCloud, pointXYZBGRMap);
    viewPCL(colorPointCloud);
    writer.write(colorPointCloudPath, colorPointCloud, true);
    std::cout << "ColorPointCloud has : " << colorPointCloud.width * colorPointCloud.height
              << " data points." << std::endl;

    device.disconnect();
    std::cout << "Disconnect Mech-Eye Success." << std::endl;

    return 0;
}
