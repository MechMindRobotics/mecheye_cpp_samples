# SetUserSets Sample Program

With this sample program, you can perform functions related to parameter groups, such as getting the names of available parameter groups, switching parameter group, and saving the current parameter values to a specific parameter group. The parameter group feature allows user to save and quickly apply a set of parameter values.

## Build the Sample Program

Prerequisites and instructions for building the sample program on Windows and Ubuntu are provided.

### Windows

#### Prerequisites

The following software are required to build this sample program. Please download and install these software.

* [Mech-Eye SDK (latest version)](https://www.mech-mind.com/download/softwaredownloading.html)
* [Visual Studio (version 2015 or above)](https://visualstudio.microsoft.com/vs/community/)
* [CMake (version 3.2 or above)](https://cmake.org/download/)

#### Instructions

1. Make sure the `SetUserSets` folder is in a location with read and write permissions.
2. Run Cmake and set the source and build paths:
   
   | Field                       | Path                  |
   | :----                       | :----                 |
   | Where is the source code    | xxx/SetUserSets       |
   | Where to build the binaries | xxx/SetUserSets/build |

3. Click the **Configure** button. In the pop-up window, set the generator and platform according to the actual situation, and then click the **Finish** button.
4. When the log displays **Configuring done**, click the **Generate** button. When the log displays **Generating done**, click the **Open Project** button.
5. In Visual Studio, change the Solution Configuration from **Debug** to **Release**.
6. Right-click the sample in **Solution Explorer**, and select **Set as Startup Project**.
7. Click **Local Windows Debugger** to build the solution.
8. Enter the index of the camera to which you want to connect, and press the Enter key. The obtained files are saved to the `build` folder.

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

#### Instructions

1. Navigate to the `SetUserSets` folder. 
   
   ```bash
   cd /opt/mech-mind/mech-eye-sdk/samples/Util/SetUserSets/
   ```

2. Configure and build the sample program.

   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

3. Run the sample program.

   ```bash
   ./SetUserSets
   ```
   
4. Enter the index of the camera to which you want to connect, and press the Enter key.