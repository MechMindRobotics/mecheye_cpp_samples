# C++ Samples

This documentation provides descriptions of Mech-Eye API C++ samples and instructions for building all the samples at once.

If you have any questions or have anything to share, feel free to post on the [Mech-Mind Online Community](https://community.mech-mind.com/). The community also contains a [specific category for development with Mech-Eye SDK](https://community.mech-mind.com/c/mech-eye-sdk-development/19).

## Sample List

Samples are divided into the following categories: **Basic**, **Advanced**, **Util**, **Laser**, **UHP**, **Calibration** and **Halcon**.

- **Basic**: camera connection and basic capturing functions.
- **Advanced**: advanced capturing functions.
- **Util**: obtain information from a camera and set camera parameters.
- **Laser**: for Laser, LSR and DEEP series cameras only.
- **UHP**: for UHP series cameras only. 
- **Calibration**: for performing hand-eye calibration.
- **Halcon**: obtain HALCON-readable point clouds via Mech-Eye API, not available on Arm-based platforms.

The samples marked with `(OpenCV)` require [OpenCV](https://opencv.org/releases/) to be installed.  
The samples marked with `(PCL)` require [PCL](https://github.com/PointCloudLibrary/pcl/releases) to be installed.  
The sample marked with `(HALCON)` requires [HALCON](https://www.mvtec.com/downloads) to be installed.

- **Basic**
  - [ConnectToCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/ConnectToCamera)  
    Connect to a Mech-Eye Industrial 3D Camera.
  - [ConnectAndCaptureImage](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/ConnectAndCaptureImage)  
    Connect to a camera and obtain the 2D image, depth map and point cloud data.
  - [CaptureColorMap](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/CaptureColorMap) `(OpenCV)`  
    Obtain and save the 2D image in OpenCV format from a camera.
  - [CaptureDepthMap](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/CaptureDepthMap) `(OpenCV)`  
    Obtain and save the depth map in OpenCV format from a camera.
  - [CapturePointCloud](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/CapturePointCloud) `(PCL)`  
    Obtain and save untextured and textured point clouds (PCL format) generated from images captured with a single exposure time.
  - [CaptureHDRPointCloud](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/CaptureHDRPointCloud) `(PCL)`  
    Obtain and save untextured and textured point clouds (PCL format) generated from images captured with multiple exposure times.
  - [CapturePointCloudROI](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/CapturePointCloudROI) `(PCL)`  
    Obtain and save untextured and textured point clouds (PCL format) of the objects in the ROI from a camera.
  - [CapturePointCloudFromTextureMask](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Basic/CapturePointCloudFromTextureMask) `(PCL)`  
    Construct and save untextured and textured point clouds (PCL format) generated from a depth map and masked 2D image.
- **Advanced**
  - [CaptureCloudFromDepth](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Advanced/CaptureCloudFromDepth) `(PCL)`  
    Construct and save point clouds (PCL format) from the depth map and 2D image obtained from a camera.
  - [CaptureSequentiallyMultiCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Advanced/CaptureSequentiallyMultiCamera) `(OpenCV & PCL)`  
    Obtain and save 2D images, depth maps and point clouds sequentially from multiple cameras.
  - [CaptureSimultaneouslyMultiCamera](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Advanced/CaptureSimultaneouslyMultiCamera) `(OpenCV & PCL)`  
    Obtain and save 2D images, depth maps and point clouds simultaneously from multiple cameras.
  - [CaptureTimedAndPeriodically](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Advanced/CaptureTimedAndPeriodically) `(OpenCV & PCL)`  
    Obtain and save 2D images, depth maps and point clouds periodically for the specified duration from a camera.
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
    Perform functions related to parameter groups, such as getting the names of available parameter groups, switching parameter group, and saving the current parameter values to a specific parameter group. The parameter group feature allows user to save and quickly apply a set of parameter values.
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
    Set the capture mode (capture images with 2D camera 1, with 2D camera 2, or with both 2D cameras and compose the outputs).
  - [SetUHPFringeCodingMode](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/UHP/SetUHPFringeCodingMode)  
    Set the coding mode of the structured light pattern.
- **Calibration**
  - [HandEyeCalibration](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Calibration/HandEyeCalibration)
    Perform hand-eye calibration.
- **Halcon**
  - [CaptureHalconPointCloud](https://github.com/MechMindRobotics/mecheye_cpp_samples/tree/master/source/Halcon/CaptureHalconPointCloud) `(PCL & HALCON)`  
  Obtain point cloud data from a camera, and then transform and save the point clouds using HALCON C++ interface. Not available on ARM-based platforms.

## Build the Sample Programs

The instructions provided here allow you to build all the sample programs at once.

> Note: The CaptureHalconPointCloud sample program is not available on Arm-based platforms.

### Windows

#### Prerequisites

The following software are required to build the sample programs. Please download and install these software.

* [Mech-Eye SDK (latest version)](https://community.mech-mind.com/c/latest-product-downloads/10)
* [Visual Studio (version 2017 or above)](https://visualstudio.microsoft.com/vs/community/)
* [CMake (version 3.2 or above)](https://cmake.org/download/)

Optional software: If you need to build the sample programs dependent on third-party software (refer to the Sample List above), please install the corresponding software.

* [HALCON (version 20.11 or above)](https://www.mvtec.com/downloads)
  
  > Note: HALCON versions below 20.11 are not fully tested.

* [OpenCV (version 3.4.5 or above)](https://opencv.org/releases/)
* [PCL (version 1.8.1 or above)](https://github.com/PointCloudLibrary/pcl/releases): Refer to the following table and determine the version of PCL to install based on the version of Visual Studio. Download the EXE installer from the **Assets** section of the version that you want to install. 

   | Visual Studio version       | Supported PCL versions         |
   | :----                       | :----                          |
   | 2017                        | 1.8.1–1.9.1                    |
   | 2019                        | 1.8.1–1.12.1                   |
   | 2022                        | 1.8.1 and above                |

  > Note: PCL is not supported in Visual Studio 2017.

#### Instructions

1. Make sure the sample folders are in a location with read and write permissions.
2. If OpenCV and/or PCL are installed, add the following directories to the **Path** environment variable:
   
   * For PCL: `C:\Program Files\OpenNI\Tools`
   * For OpenCV: `xxx\opencv\build\x64\vc14\bin`
   * For OpenCV: `xxx\opencv\build\x64\vc14\lib`

3. Disable unneeded sample programs: if any of the optional software are not installed, this step must be performed.
   
   Open the CMakeLists file in `xxx\Mech-Eye SDK-x.x.x\API\samples`, and change **ON** to **OFF** in the options of the unneeded sample programs.

4. Run Cmake and set the source and build paths: 
   
   | Field                       | Path                                     |
   | :----                       | :----                                    |
   | Where is the source code    | xxx\Mech-Eye SDK-x.x.x\API\samples       |
   | Where to build the binaries | xxx\Mech-Eye SDK-x.x.x\API\samples\build |

5. Click the **Configure** button. In the pop-up window, set the generator and platform according to the actual situation, and then click the **Finish** button.
6. When the log displays **Configuring done**, click the **Generate** button. When the log displays **Generating done**, click the **Open Project** button.
7. In Visual Studio, change the Solution Configuration from **Debug** to **Release**.
8. Right-click **Solution 'MechEyeCppSamples'** in **Solution Explorer**, and select **Build Solution**.
9. Navigate to the `Release` folder under the **Where to build the binaries** directory, and run a sample program.
10. Enter the index of the camera to which you want to connect, and press the Enter key. The obtained files are saved to the `Release` folder.

### Ubuntu

Ubuntu 18 or above is required.

#### Prerequisites

* Update the software source list.
  
  ```bash
  sudo apt-get update
  ```

* Install required tools.
  
  ```bash
  sudo apt-get install -y build-essential pkg-config cmake
  ```

* Install [Mech-Eye API (latest version)](https://community.mech-mind.com/c/latest-product-downloads/10).
* Install optional third-party libraries: If you need to build the sample programs dependent on third-party software (refer to the Sample List above), please install the corresponding software.
  
  * Install OpenCV (latest version):
    
    ```bash
    sudo apt update && sudo apt install -y unzip
    wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
    unzip opencv.zip
    mkdir build && cd build
    cmake ../opencv-4.x
    cmake --build .
    sudo make install
    ```
  
  * Install PCL: 
    
    ```bash
    sudo apt-get install libpcl-dev
    ```

    > Note: On different versions of Ubuntu, this command installs different versions of PCL. On Ubuntu 18.04, PCL 1.8.1 is installed; on Ubunt 20.04, PCL 1.10.0 is installed.

  * Install HALCON (20.11 or above)

    > Note: HALCON versions below 20.11 are not fully tested.

    ```bash
    tar zxvf halcon-20.11.3.0-linux.tar.gz
    sudo sh install-linux.sh #Note down the installation directory of HALCON.
    ```

  * Add persistent environment variables for HALCON: open `/etc/profile` in an editor (such as vi) and paste the following lines to the end of the file. Replace `/opt/halcon` with the actual installation directory of HALCON. 

    ```bash
    HALCONARCH=x64-linux; export HALCONARCH
    HALCONROOT="/opt/halcon"; export HALCONROOT
    HALCONEXAMPLES=${HALCONROOT}/examples; export HALCONEXAMPLES
    HALCONIMAGES=${HALCONROOT}/examples/images; export HALCONIMAGES
    PATH=${HALCONROOT}/bin/${HALCONARCH}:${PATH}; export PATH
    if [ ${LD_LIBRARY_PATH} ] ; then
       LD_LIBRARY_PATH=${HALCONROOT}/lib/${HALCONARCH}:${LD_LIBRARY_PATH}; export LD_LIBRARY_PATH
    else
       LD_LIBRARY_PATH=${HALCONROOT}/lib/${HALCONARCH}; export LD_LIBRARY_PATH
    fi
    ```
   
  > Note:
  >
  > - The changes are applied when you log in again. Or, you can `source /etc/profile/` before you configure and build the sample program.
  > - For more information, please refer to HALCON's installation guide.

#### Instructions

1. Navigate to the sample directory. 
   
   ```bash
   cd /opt/mech-mind/mech-eye-sdk/samples/
   ```
2. Disable unneeded sample programs: if any of the optional software are not installed, this step must be performed.
   
   Open the CMakeLists file in `/opt/mech-mind/mech-eye-sdk/samples/`, and change **ON** to **OFF** in the options of the unneeded sample programs.

3. Configure and build the sample programs.

   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

4. Run a sample program. Replace `SampleName` with the name of the sample that you want to run.

   ```bash
   ./SampleName
   ```

5. Enter the index of the camera to which you want to connect, and press the Enter key. The obtained files are saved to `/opt/mech-mind/mech-eye-sdk/samples/build`.


## License

Mech-Eye Samples are distributed under the [BSD license](https://github.com/MechMindRobotics/mecheye_cpp_samples/blob/master/LICENSE).