# CaptureHalconPointCloud Sample Program

With this sample program, you can obtain point cloud data from a camera, and then transform and save the point clouds using HALCON C++ interface.

> Note: This sample program is not available on Arm-based platforms.

If you have any questions or have anything to share, feel free to post on the [Mech-Mind Online Community](https://community.mech-mind.com/). The community also contains a [specific category for development with Mech-Eye SDK](https://community.mech-mind.com/c/mech-eye-sdk-development/19).

## Build the Sample Program

Prerequisites and instructions for building the sample program on Windows and Ubuntu are provided.

### Windows

#### Prerequisites

The following software are required to build this sample program. Please download and install these software.

* [Mech-Eye SDK (latest version)](https://community.mech-mind.com/c/latest-product-downloads/10)
* [Visual Studio (version 2017 or above)](https://visualstudio.microsoft.com/vs/community/)
* [CMake (version 3.2 or above)](https://cmake.org/download/)
* [HALCON (version 20.11 or above)](https://www.mvtec.com/downloads)

  > Note: HALCON versions below 20.11 are not fully tested.

#### Instructions

1. Make sure the `CaptureHalconPointCloud` folder is in a location with read and write permissions.
2. Run Cmake and set the source and build paths:
   
   | Field                       | Path                              |
   | :----                       | :----                             |
   | Where is the source code    | xxx\CaptureHalconPointCloud       |
   | Where to build the binaries | xxx\CaptureHalconPointCloud\build |

3. Click the **Configure** button. In the pop-up window, set the generator and platform according to the actual situation, and then click the **Finish** button.
4. When the log displays **Configuring done**, click the **Generate** button. When the log displays **Generating done**, click the **Open Project** button.
5. In Visual Studio, change the Solution Configuration from **Debug** to **Release**.
6. Right-click the sample in **Solution Explorer**, and select **Set as Startup Project**.
7. Click **Local Windows Debugger** to build the solution.
8. Enter the index of the camera to which you want to connect, and press the Enter key. The obtained files are saved to the `build` folder.
9. Read the point cloud files into HDevelop with the `read_object_model_3d` operator.

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
* Install third-party libraries: PCL and HALCON are required. 
    
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

1. Navigate to the `CaptureHalconPointCloud` folder.
   
   ```bash
   cd /opt/mech-mind/mech-eye-sdk/samples/Advanced/CaptureHalconPointCloud/

   ```

2. Configure and build the sample program.

   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

3. Run the sample program.

   ```bash
   ./CaptureHalconPointCloud
   ```

4. Enter the index of the camera to which you want to connect, and press the Enter key. The obtained files are saved to `/CaptureCloudFromDepth/build`.
5. Read the point cloud files into HDevelop with the `read_object_model_3d` operator.