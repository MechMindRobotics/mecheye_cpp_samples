# CaptureHDRPointCloud Sample Program

With this sample program, you can obtain and save untextured and textured point clouds (PCL format) generated from images captured with multiple exposure times.

If you have any questions or have anything to share, feel free to post on the [Mech-Mind Online Community](https://community.mech-mind.com/). The community also contains a [specific category for development with Mech-Eye SDK](https://community.mech-mind.com/c/mech-eye-sdk-development/19).

## Build the Sample Program

Prerequisites and instructions for building the sample program on Windows and Ubuntu are provided.

### Windows

#### Prerequisites

The following software are required to build this sample program. Please download and install these software.

* [Mech-Eye SDK (latest version)](https://community.mech-mind.com/c/latest-product-downloads/10)
* [Visual Studio (version 2017 or above)](https://visualstudio.microsoft.com/vs/community/)
* [CMake (version 3.2 or above)](https://cmake.org/download/)
* [PCL (version 1.8.1 or above)](https://github.com/PointCloudLibrary/pcl/releases): Refer to the following table and determine the version of PCL to install based on the version of Visual Studio. Download the EXE installer from the **Assets** section of the version that you want to install. 

   | Visual Studio version       | Supported PCL versions         |
   | :----                       | :----                          |
   | 2017                        | 1.8.1–1.9.1                    |
   | 2019                        | 1.8.1–1.12.1                   |
   | 2022                        | 1.8.1 and above                |

#### Instructions

1. Make sure the `CaptureHDRPointCloud` folder is in a location with read and write permissions.
2. Add the following directories to the **Path** environment variable:
   
   * `C:\Program Files\OpenNI\Tools`

3. Run Cmake and set the source and build paths:
   
   | Field                       | Path                           |
   | :----                       | :----                          |
   | Where is the source code    | xxx\CaptureHDRPointCloud       |
   | Where to build the binaries | xxx\CaptureHDRPointCloud\build |

4. Click the **Configure** button. In the pop-up window, set the generator and platform according to the actual situation, and then click the **Finish** button.
5. When the log displays **Configuring done**, click the **Generate** button. When the log displays **Generating done**, click the **Open Project** button.
6. In Visual Studio, change the Solution Configuration from **Debug** to **Release**.
7. Right-click the sample in **Solution Explorer**, and select **Set as Startup Project**.
8. Click **Local Windows Debugger** to build the solution.
9. Enter the index of the camera to which you want to connect, and press the Enter key. The obtained files are saved to the `build` folder.

### Ubuntu

Ubuntu 18 or above is required.

#### Prerequisites

* Update the software source list.
  
  ```
  sudo apt-get update
  ```

* Install required tools.
  
  ```
  sudo apt-get install -y build-essential pkg-config cmake
  ```

* Install [Mech-Eye API (latest version)](https://community.mech-mind.com/c/latest-product-downloads/10).
* Install third-party libraries: PCL is required.
  
  * Install PCL: 
    
    ```
    sudo apt-get install libpcl-dev
    ```

#### Instructions

1. Navigate to the `CaptureHDRPointCloud` folder. 
   
   ```
   cd /opt/mech-mind/mech-eye-sdk/samples/Basic/CaptureHDRPointCloud/
   ```
   
2. Configure and build the sample program.

   ```
   mkdir build && cd build
   cmake ..
   make
   ```

3. Run the sample program.

   ```
   ./CaptureHDRPointCloud
   ```
   
4. Enter the index of the camera to which you want to connect, and press the Enter key. The obtained files are saved to `/CaptureHDRPointCloud/build`.