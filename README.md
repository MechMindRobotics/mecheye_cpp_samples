# C++ Samples

This repository contains C++ samples for Mech-Eye SDK.

## Installation

1. Download and install [Mech-Eye SDK](https://www.mech-mind.com/download/camera-sdk.html)
2. Clone this repository to a specific folder.
3. Configure the sample solution with CMake, open it in Visual Studio, build and run it.

## Sample list

There are four categoires of samples: **Basic**, **Advanced**, **Util**, and **Laser**.  

The category **Basic** contains samples that are related to basic connecting and capturing.  
The category **Advanced** contains samples that use advanced capturing tricks.  
The category **Util** contains samples that get and print information and set parameters.  
The category **Laser** contains samples that can only be used on laser cameras.  

The samples marked with `(OpenCV)` require [OpenCV](https://opencv.org/releases/) to be installed.  
The samples marked with `(PCL)` require [PCL](https://github.com/PointCloudLibrary/pcl/releases) to be installed.

- **Basic**
  - [ConnectToCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/connectToCamera)  
    Connects to a Mech-Eye Industrial 3D Camera.
  - [ConnectAndCaptureImage](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/connectAndCaptureImage)  
    Connects to a Mech-Eye Industrial 3D Camera and capture 2D and 3D data.
  - [CaptureColorMap](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/captureColorMap) `(OpenCV)`  
    Capture color maps data with OpenCV data structure from a camera.
  - [CaptureDepthMap](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/captureDepthMap) `(OpenCV)`  
    Capture depth maps data with OpenCV data structure from a camera.
  - [CapturePointCloud](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/capturePointCloud) `(PCL)`  
    Capture point clouds with PCL data structure from a camera.
  - [CaptureHDRPointCloud](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/captureHDRPointCloud) `(PCL)`  
    Capture point clouds in HDR mode with PCL data structure from a camera.
  - [CapturePointCloudROI](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/capturePointCloudROI) `(PCL)`  
    Capture point clouds with ROI enabled with PCL data structure from a camera.
- **Advanced**
  - [CaptureCloudFromDepth](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Advanced/captureCloudFromDepth) `(PCL)`  
    Construct point clouds from depth map and color map captured from a camera.
  - [CaptureSequentiallyMultiCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Advanced/captureSequentiallyMultiCamera) `(OpenCV & PCL)`  
    Capture sequentially from multiple cameras.
  - [CaptureSimultaneouslyMultiCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Advanced/captureSimultaneouslyMultiCamera) `(OpenCV & PCL)`  
    Capture simultaneously from multiple cameras.
  - [CaptureTimedAndPeriodically](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Advanced/captureTimedAndPeriodically) `(OpenCV & PCL)`  
    Capture periodically for a specific time form a camera.
- **Util**
  - [GetCameraIntri](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/getCameraIntri)  
    Get and print a camera's intrinsic parameters.
  - [PrintDeviceInfo](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/printDeviceInfo)  
    Get and print a camera's information.
  - [SetDepthRange](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/setDepthRange)  
    Set the depth range of a camera.
  - [SetUserSets](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/setUserSets)  
    Get current user set name and available user sets, save settings to a specific user set.
  - [SetAndGetParameter](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/setAndGetParameter)  
    Set and get the settings and usersets from a camera.
- **Laser**
  - [SetFramePartitionCount](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Laser/setFramePartitionCount)  
    Set the laser scan partition number for a laser camera.
  - [SetFrameRange](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Laser/setFrameRange)  
    Set the laser scan field of view for a laser camera.
  - [SetFringeCodingMode](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Laser/setFringeCodingMode)  
    Set the fringe coding mode for a laser camera.
  - [SetPowerLevel](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Laser/setPowerLevel)  
    Set the power level for a laser camera.

## License

Mech-Eye Samples are distributed under the [BSD license](https://github.com/MechMindRobotics/mecheye_cpp_samples/blob/main/LICENSE)
