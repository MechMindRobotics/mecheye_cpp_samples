#include <iostream>
#include <thread>

#include "MechEyeApi.h"
#include "SampleUtil.h"
#include "PclUtil.h"

int main()
{
    mmind::api::MechEyeDevice device;
    if (!findAndConnect(device))
        return -1;

    // Set 3D Exposure Sequence
    showError(device.setScan3DExposure(std::vector<double>{5, 10}));

    mmind::api::PointXYZMap pointXYZMap;
    showError(device.capturePointXYZMap(pointXYZMap));

    mmind::api::PointXYZBGRMap pointXYZBGRMap;
    showError(device.capturePointXYZBGRMap(pointXYZBGRMap));

    std::string pointCloudPath = "PointCloudXYZ.ply";
    pcl::PointCloud<pcl::PointXYZ> pointCloud(pointXYZMap.width(), pointXYZMap.height());
    toPCL(pointCloud, pointXYZMap);
    viewPCL(pointCloud);
    savePLY(pointXYZMap, pointCloudPath);

    std::string colorPointCloudPath = "PointCloudXYZRGB.ply";
    pcl::PointCloud<pcl::PointXYZRGB> colorPointCloud(pointXYZBGRMap.width(),
                                                      pointXYZBGRMap.height());
    toPCL(colorPointCloud, pointXYZBGRMap);
    viewPCL(colorPointCloud);
    savePLY(pointXYZBGRMap, colorPointCloudPath);

    device.disconnect();
    std::cout << "Disconnect Mech-Eye Success." << std::endl;

    return 0;
}
