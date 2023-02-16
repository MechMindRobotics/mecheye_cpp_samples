# CaptureTimedAndPeriodically Sample Program

With this sample program, you can obtain and save 2D images, depth maps and point clouds periodically for the specified duration from a camera.

## Build the Sample Program

Prerequisites and instructions for building the sample program on Windows and Ubuntu are provided.

### Windows

#### Prerequisites

The following software are required to build this sample program. Please download and install these software.

* [Mech-Eye SDK (latest version)](https://www.mech-mind.com/download/softwaredownloading.html)
* [Visual Studio (version 2015 or above)](https://visualstudio.microsoft.com/vs/community/)
* [CMake (version 3.2 or above)](https://cmake.org/download/)
* [OpenCV (version 3.4.5 or above)](https://opencv.org/releases/)
* [PCL (version 1.12.1)](https://github.com/PointCloudLibrary/pcl/releases): download the **PCL-1.12.1-AllInOne-msvc2019-win64.exe** file from the **Assets** section of **PCL 1.12.1**.

#### Instructions

1. Make sure the `CaptureTimedAndPeriodically` folder is in a location with read and write permissions.
2. Add the following directories to the **Path** environment variable:
   
   * `C:\Program Files\OpenNI\Tools`
   * `xxx\opencv\build\x64\vc14\bin`
   * `xxx\opencv\build\x64\vc14\lib`

3. Run Cmake and set the source and build paths:
   
   | Field                       | Path                                  |
   | :----                       | :----                                 |
   | Where is the source code    | xxx\CaptureTimedAndPeriodically       |
   | Where to build the binaries | xxx\CaptureTimedAndPeriodically\build |

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
  
  ```bash
  sudo apt-get update
  ```

* Install required tools.
  
  ```bash
  sudo apt-get install -y build-essential pkg-config cmake
  ```

* Install [Mech-Eye API (latest version)](https://www.mech-mind.com/download/softwaredownloading.html).
* Install third-party libraries: OpenCV and PCL are required.
  
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
  
  * Install PCL (latest version): 
    
    ```bash
    sudo apt-get install libpcl-dev
    ```

#### Instructions

1. Navigate to the `CaptureTimedAndPeriodically` folder. 
   
   ```bash
   cd /opt/mech-mind/mech-eye-sdk/samples/Advanced/CaptureTimedAndPeriodically/
   ```

2. Configure and build the sample program.

   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

3. Run the sample program.

   ```bash
   ./CaptureTimedAndPeriodically
   ```

4. Enter the index of the camera to which you want to connect, and press the Enter key. The obtained files are saved to `/CaptureTimedAndPeriodically/build`.