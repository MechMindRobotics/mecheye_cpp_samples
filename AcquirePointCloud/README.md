# AcquirePointCloud Sample

With this sample, you can acquire the profile data, generate the point cloud, and save the point cloud in the CSV format.

If you have any questions or have anything to share, feel free to post on the [Mech-Mind Online Community](https://community.mech-mind.com/). The community also contains a [specific category for development with Mech-Eye SDK](https://community.mech-mind.com/c/mech-eye-sdk-development/19).

## Build the Sample

Prerequisites and instructions for building the sample on Windows are provided.

### Prerequisites

The following software are required to build this sample. Please download and install these software.

* [Mech-Eye SDK (latest version)](https://downloads.mech-mind.com/?tab=tab-sdk)
* [Visual Studio (version 2017 or above)](https://visualstudio.microsoft.com/vs/community/)
* [CMake (version 3.2 or above)](https://cmake.org/download/)

### Instructions

1. Make sure that the sample is stored in a location with read and write permissions.
2. Run Cmake and set the source and build paths:
   
   | Field                       | Path                  |
   | :----                       | :----                 |
   | Where is the source code    | xxx/AcquirePointCloud       |
   | Where to build the binaries | xxx/AcquirePointCloud/build |

3. Click the **Configure** button. In the pop-up window, set the generator and platform according to the actual situation, and then click the **Finish** button.
4. When the log displays **Configuring done**, click the **Generate** button. When the log displays **Generating done**, click the **Open Project** button.
5. In Visual Studio, change the Solution Configuration from **Debug** to **Release**.
6. Right-click the sample in **Solution Explorer**, and select **Set as Startup Project**.
7. Click **Local Windows Debugger** to build the solution.
8. Enter the index of the camera to which you want to connect, and press the Enter key. The obtained files are saved to the `build` folder.