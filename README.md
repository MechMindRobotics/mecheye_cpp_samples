# C++ Samples

This documentation provides descriptions of Mech-Eye API C++ samples and instructions for building all the samples at once.

If you have any questions or have anything to share, feel free to post on the [Mech-Mind Online Community](https://community.mech-mind.com/). The community also contains a [specific category for development with Mech-Eye SDK](https://community.mech-mind.com/c/mech-eye-sdk-development/19).

## Sample List

Currently, the following samples are provided.

The samples marked with `(OpenCV)` require [OpenCV](https://opencv.org/releases/) to be installed.  

- **AcquireProfileData** `(OpenCV)`
  Acquires the profile data, generates the intensity image and depth map, and saves the images.
- **AcquireProfileDataUsingCallback** `(OpenCV)`
  Acquires the profile data using a callback function, generates the intensity image and depth map, and saves the images.
- **AcquireRawImage** `(OpenCV)`
  Acquires and saves the raw image.
- **AcquirePointCloud**
  Acquires the profile data, generates the point cloud, and saves the point cloud in the CSV format.
- **ManageUserSets**
  Performs functions related to parameter groups, such as obtaining the names of all available parameter groups, selecting a parameter group, and saving the parameter values to the current parameter group. The parameter group feature allows user to save and quickly apply a set of parameter values.

## Build the Samples

The instructions provided here allow you to build all the samples at once.

### Prerequisites

The following software are required to build the samples. Please download and install these software.

* [Mech-Eye SDK (latest version)](https://downloads.mech-mind.com/?tab=tab-sdk)
* [Visual Studio (version 2017 or above)](https://visualstudio.microsoft.com/vs/community/)
* [CMake (version 3.2 or above)](https://cmake.org/download/)

Optional software: If you need to build the samples dependent on third-party software (refer to the Sample List above), please install the corresponding software.

* [OpenCV (version 3.4.5 or above)](https://opencv.org/releases/)

### Instructions

1. Make sure that the samples are stored in a location with read and write permissions
2. If OpenCV is installed, add the following directories to the **Path** environment variable:
   
   * `xxx\opencv\build\x64\vc14\bin`
   * `xxx\opencv\build\x64\vc14\lib`

3. Disable unneeded samples: if OpenCV is installed, this step must be performed.
   
   Open the CMakeLists file in `xxx\source`, and change **ON** to **OFF** in the line that starts with `option(USE_OPENCV`.

4. Run Cmake and set the source and build paths: 
   
   | Field                       | Path                 |
   | :----                       | :----                |
   | Where is the source code    | xxx\source           |
   | Where to build the binaries | xxx\source\build     |

5. Click the **Configure** button. In the pop-up window, set the generator and platform according to the actual situation, and then click the **Finish** button.
6. When the log displays **Configuring done**, click the **Generate** button. When the log displays **Generating done**, click the **Open Project** button.
7. In Visual Studio, change the Solution Configuration from **Debug** to **Release**.
8. Right-click **Solution 'MechEyeCppSamples'** in **Solution Explorer**, and select **Build Solution**.
9. Navigate to the `Release` folder under the **Where to build the binaries** directory, and run a sample.
10. Enter the index of the camera to which you want to connect, and press the Enter key. The obtained files are saved to the `Release` folder.