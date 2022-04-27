# C++ Samples

This repository contains C++ samples for Mech-Eye SDK.

## Installation

1. Download and install [Mech-Eye SDK](https://www.mech-mind.com/download/camera-sdk.html).
2. Clone this repository to a specific folder.
3. Configure the sample solution with CMake, open it in Visual Studio, and build and run it.

## Sample list

There are four categories of samples: **Basic**, **Advanced**, **Util**, and **Laser**.  

- The category **Basic** contains samples that are related to basic connecting and capturing.  
- The category **Advanced** contains samples that use advanced capturing tricks.  
- The category **Util** contains samples that get and print information and set parameters.  
- The category **Laser** contains samples that can only be used on Mech-Eye Laser cameras.  

The samples marked with `(OpenCV)` require [OpenCV](https://opencv.org/releases/) to be installed.  
The samples marked with `(PCL)` require [PCL](https://github.com/PointCloudLibrary/pcl/releases) to be installed.

- **Basic**
  - [ConnectToCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/connectToCamera)  
    Connect to a Mech-Eye Industrial 3D Camera.
  - [ConnectAndCaptureImage](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/connectAndCaptureImage)  
    Connect to a Mech-Eye Industrial 3D Camera and capture 2D and 3D data.
  - [CaptureColorMap](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/captureColorMap) `(OpenCV)`  
    Capture color image data with OpenCV data structure from a camera.
  - [CaptureDepthMap](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/captureDepthMap) `(OpenCV)`  
    Capture depth map data with OpenCV data structure from a camera.
  - [CapturePointCloud](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/capturePointCloud) `(PCL)`  
    Capture monochrome and color point clouds with PCL data structure from a camera.
  - [CaptureHDRPointCloud](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/captureHDRPointCloud) `(PCL)`  
    Capture monochrome and color point clouds in HDR mode with PCL data structure from a camera.
  - [CapturePointCloudROI](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Basic/capturePointCloudROI) `(PCL)`  
    Capture monochrome and color point clouds in ROI with PCL data structure from a camera.
- **Advanced**
  - [CaptureCloudFromDepth](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Advanced/captureCloudFromDepth) `(PCL)`  
    Construct point clouds from depth map and color image data captured from a camera.
  - [CaptureSequentiallyMultiCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Advanced/captureSequentiallyMultiCamera) `(OpenCV & PCL)`  
    Capture sequentially from multiple cameras.
  - [CaptureSimultaneouslyMultiCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Advanced/captureSimultaneouslyMultiCamera) `(OpenCV & PCL)`  
    Capture simultaneously from multiple cameras.
  - [CaptureTimedAndPeriodically](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Advanced/captureTimedAndPeriodically) `(OpenCV & PCL)`  
    Capture periodically for a specific time from a camera.
- **Util**
  - [GetCameraIntri](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/getCameraIntri)  
    Get and print a camera's intrinsic parameters.
  - [PrintDeviceInfo](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/printDeviceInfo)  
    Get and print a camera's information.
  - [SetDepthRange](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/setDepthRange)  
    Set the depth range of a camera.
  - [SetUserSets](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/setUserSets)  
    Get the current userset name and available usersets of parameter settings, and save the settings to a specific userset.
  - [SetAndGetParameter](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Util/setAndGetParameter)  
    Set a specified parameter to a camera and print it.
- **Laser**
  - [SetFramePartitionCount](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Laser/setFramePartitionCount)  
    Set the laser scan partition count for a Mech-Eye Laser camera.
  - [SetFrameRange](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Laser/setFrameRange)  
    Set the laser scan range for a Mech-Eye Laser camera.
  - [SetFringeCodingMode](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Laser/setFringeCodingMode)  
    Set the fringe coding mode for a Mech-Eye Laser camera.
  - [SetPowerLevel](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/Laser/setPowerLevel)  
    Set the laser power level for a Mech-Eye Laser camera.

## License

Mech-Eye Samples are distributed under the [BSD license](https://github.com/MechMindRobotics/mecheye_cpp_samples/blob/main/LICENSE).
