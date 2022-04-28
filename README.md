# C++ Samples

This repository contains C++ samples for Mech-Eye SDK.

## Installation

1. Download and install [Mech-Eye SDK](https://www.mech-mind.com/download/camera-sdk.html).
2. Clone this repository to a specific folder.
3. Configure the sample solution with CMake, open it in Visual Studio, build and run it.

## Sample list

There are four categories of samples: **Basic**, **Advanced**, **Util**, and **Laser**.  

- The category **Basic** contains samples that are related to basic connecting and capturing.  
- The category **Advanced** contains samples that use advanced capturing tricks.  
- The category **Util** contains samples that get and print information and set parameters.  
- The category **Laser** contains samples that can only be used on Mech-Eye Laser cameras.  

The samples marked with `(OpenCV)` require [OpenCV](https://opencv.org/releases/) to be installed.  
The samples marked with `(PCL)` require [PCL](https://github.com/PointCloudLibrary/pcl/releases) to be installed.

- **Basic**
  - [ConnectToCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/ConnectToCamera)  
    Connect to a Mech-Eye Industrial 3D Camera.
  - [ConnectAndCaptureImage](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/ConnectAndCaptureImage)  
    Connect to a Mech-Eye Industrial 3D Camera and capture 2D and 3D data.
  - [CaptureColorMap](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/CaptureColorMap) `(OpenCV)`  
    Capture color image data with OpenCV data structure from a camera.
  - [CaptureDepthMap](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/CaptureDepthMap) `(OpenCV)`  
    Capture depth map data with OpenCV data structure from a camera.
  - [CapturePointCloud](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/CapturePointCloud) `(PCL)`  
    Capture monochrome and color point clouds with PCL data structure from a camera.
  - [CaptureHDRPointCloud](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/CaptureHDRPointCloud) `(PCL)`  
    Capture monochrome and color point clouds in HDR mode with PCL data structure from a camera.
  - [CapturePointCloudROI](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/CapturePointCloudROI) `(PCL)`  
    Capture monochrome and color point clouds in ROI with PCL data structure from a camera.
- **Advanced**
  - [CaptureCloudFromDepth](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Advanced/CaptureCloudFromDepth) `(PCL)`  
    Construct point clouds from depth map and color image data captured from a camera.
  - [CaptureSequentiallyMultiCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Advanced/CaptureSequentiallyMultiCamera) `(OpenCV & PCL)`  
    Capture sequentially from multiple cameras.
  - [CaptureSimultaneouslyMultiCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Advanced/CaptureSimultaneouslyMultiCamera) `(OpenCV & PCL)`  
    Capture simultaneously from multiple cameras.
  - [CaptureTimedAndPeriodically](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Advanced/CaptureTimedAndPeriodically) `(OpenCV & PCL)`  
    Capture periodically for a specific time from a camera.
- **Util**
  - [GetCameraIntri](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/GetCameraIntri)  
    Get and print a camera's intrinsic parameters.
  - [PrintDeviceInfo](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/PrintDeviceInfo)  
    Get and print a camera's information.
  - [SetDepthRange](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/SetDepthRange)  
    Set the depth range of a camera.
  - [SetParameters](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/SetParameters)  
    Set specified parameters to a camera.
  - [SetUserSets](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/SetUserSets)  
    Get the current userset name and available usersets of parameter settings, and save the settings to a specific userset. The User Set feature allows the user to customize and store the individual settings.
- **Laser**
  - [SetLaserFramePartitionCount](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Laser/SetLaserFramePartitionCount)  
    Set the laser scan partition count for a Mech-Eye Laser camera.
  - [SetLaserFrameRange](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Laser/SetLaserFrameRange)  
    Set the laser scan range for a Mech-Eye Laser camera.
  - [SetLaserFringeCodingMode](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Laser/SetLaserFringeCodingMode)  
    Set the fringe coding mode for a Mech-Eye Laser camera.
  - [SetLaserPowerLevel](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Laser/SetLaserPowerLevel)  
    Set the laser power level for a Mech-Eye Laser camera.

## License

Mech-Eye Samples are distributed under the [BSD license](https://github.com/MechMindRobotics/mecheye_cpp_samples/blob/main/LICENSE).
