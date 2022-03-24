# C++ Samples

This repository contains C++ samples for Mech-Eye SDK_1.5.0.

## Installation

1. Download and install [Mech-Eye SDK_1.5.0](https://www.mech-mind.com/download/CameraSDK.html)
2. Clone this repository to a specific folder.
3. Configure the sample solution with CMake, open it in Visual Studio, build it, run it.

## Sample list

- **[ConnectAndCaptureImage](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/connectAndCaptureImage)**

  Connect to the Mech-Eye Camera and capture 2D and 3D data.
- **[SetAndGetParameter](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/setAndGetParameter)**

  Set and get the settings and usersets from the Mech-Eye Camera.
- **[CaptureResultToOpenCV](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/captureResultToOpenCV)**

  Capture 2D and 3D data with OpenCV data structure from the Mech-Eye Camera.
  > This sample requires [OpenCV 3.4.5](https://opencv.org/releases/) to be installed.
- **[CaptureResultToPLY](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/main/source/captureResultToPLY)**

  Capture 2D and 3D data with PCL data structure from the Mech-Eye Camera.
  > This sample requires [PCL 1.12.1 AllInOne](https://github.com/PointCloudLibrary/pcl/releases) to be installed.

## License

Mech-Eye Samples are distributed under the [BSD license](https://github.com/MechMindRobotics/mecheye_cpp_samples/blob/main/LICENSE)