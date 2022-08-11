# C++ Samples

This repository contains C++ samples for Mech-Eye SDK.

## Installation

1. Download and install [Mech-Eye SDK](https://www.mech-mind.com/download/camera-sdk.html).
2. Clone this repository to a specific folder.
3. Configure the sample solution with CMake, open it in Visual Studio, build it, and run it.
     For software requirements, please refer to [Mech-Eye API User Manual](https://docs.mech-mind.net/latest/en-GB/MechEye/MechEyeAPI/Samples/Samples.html)

## Sample List

Samples are divided into five categories, **Basic**, **Advanced**, **Util**, **Laser** and **UHP**.

- **Basic**: camera connection and basic capturing functions.
- **Advanced**: advanced capturing functions.
- **Util**: obtain information from a camera and set camera parameters.
- **Laser**: for Laser/LSR series cameras only.
- **UHP**: for UHP series cameras only. 

The samples marked with `(OpenCV)` require [OpenCV](https://opencv.org/releases/) to be installed.  
The samples marked with `(PCL)` require [PCL](https://github.com/PointCloudLibrary/pcl/releases) to be installed.

- **Basic**
  - [ConnectToCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/ConnectToCamera)  
    Connect to a Mech-Eye Industrial 3D Camera.
  - [ConnectAndCaptureImage](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/ConnectAndCaptureImage)  
    Connect to a camera and obtain 2D image, depth map and 3D image.
  - [CaptureColorMap](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/CaptureColorMap) `(OpenCV)`  
    Obtain 2D image in OpenCV format from a camera.
  - [CaptureDepthMap](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/CaptureDepthMap) `(OpenCV)`  
    Obtain depth map in OpenCV format from a camera.
  - [CapturePointCloud](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/CapturePointCloud) `(PCL)`  
    Obtain untextured and textured point clouds (PCL format) generated from images captured with a single exposure time.
  - [CaptureHDRPointCloud](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/CaptureHDRPointCloud) `(PCL)`  
    Obtain untextured and textured point clouds (PCL format) generated from images captured with multiple exposure times.
  - [CapturePointCloudROI](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/CapturePointCloudROI) `(PCL)`  
    Obtain untextured and textured point clouds (PCL format) of the objects in the ROI from a camera.
- **Advanced**
  - [CaptureCloudFromDepth](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Advanced/CaptureCloudFromDepth) `(PCL)`  
    Construct point clouds from depth map and 2D image captured from a camera.
  - [CaptureSequentiallyMultiCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Advanced/CaptureSequentiallyMultiCamera) `(OpenCV & PCL)`  
    Obtain 2D image, depth map and 3D images sequentially from multiple cameras.
  - [CaptureSimultaneouslyMultiCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Advanced/CaptureSimultaneouslyMultiCamera) `(OpenCV & PCL)`  
    Obtain 2D image, depth map and 3D images simultaneously from multiple cameras.
  - [CaptureTimedAndPeriodically](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Advanced/CaptureTimedAndPeriodically) `(OpenCV & PCL)`  
    Obtain 2D image, depth map and 3D images periodically for the specified duration from a camera.
- **Util**
  - [GetCameraIntri](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Util/GetCameraIntri)  
    Get and print a camera's intrinsic parameters.
  - [PrintDeviceInfo](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Util/PrintDeviceInfo)  
    Get and print a camera's information such as model, serial number and firmware version.
  - [SetDepthRange](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Util/SetDepthRange)  
    Set the range of depth values to be retained by a camera.
  - [SetParameters](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Util/SetParameters)  
    Set specified parameters to a camera.
  - [SetUserSets](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Util/SetUserSets)  
    Functions related to parameter groups, such as getting the names of available parameter groups, switching parameter group, and save the current parameter values to a specific parameter group. The parameter group feature allows user to save and quickly apply a set of parameter values.
- **Laser**
  - [SetLaserFramePartitionCount](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Laser/SetLaserFramePartitionCount)  
    Divide the projector FOV into partitions and project structured light in one partition at a time. The output of the entire FOV is composed from images of all partitions.
  - [SetLaserFrameRange](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Laser/SetLaserFrameRange)  
    Set the projection range of the structured light. The entire projector FOV is from 0 to 100.
  - [SetLaserFringeCodingMode](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Laser/SetLaserFringeCodingMode)  
    Set the coding mode of the structured light pattern.
  - [SetLaserPowerLevel](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Laser/SetLaserPowerLevel)  
    Set the output power of the laser projector in percentage of max power. This affects the intensity of the laser light.
- **UHP**
  - [SetUHPCaptureMode](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/UHP/SetUHPCaptureMode)  
    set the capture mode (capture images with camera 1, with camera 2, or with both 2D cameras and compose the outputs).
  - [SetUHPFringeCodingMode](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/UHP/SetUHPFringeCodingMode)  
    set the coding mode of the structured light pattern.

## License

Mech-Eye Samples are distributed under the [BSD license](https://github.com/MechMindRobotics/mecheye_cpp_samples/blob/master/LICENSE).
