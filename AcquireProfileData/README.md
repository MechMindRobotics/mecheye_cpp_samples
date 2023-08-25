# AcquireProfileData Sample

With this sample, you can acquire the profile data, generate the intensity image and depth map, and save the images.

If you have any questions or have anything to share, feel free to post on the [Mech-Mind Online Community](https://community.mech-mind.com/). The community also contains a [specific category for development with Mech-Eye SDK](https://community.mech-mind.com/c/mech-eye-sdk-development/19).

## Build the Sample

Prerequisites and instructions for building the sample on Windows are provided.

### Prerequisites

The following software are required to build this sample. Please download and install these software.

* [Mech-Eye SDK (latest version)](https://downloads.mech-mind.com/?tab=tab-sdk)
* [Visual Studio (version 2017 or above)](https://visualstudio.microsoft.com/vs/community/)
* [CMake (version 3.2 or above)](https://cmake.org/download/)
* [OpenCV (version 3.4.5 or above)](https://opencv.org/releases/)

### Instructions

1. Make sure that the sample is stored in a location with read and write permissions.
2. Add the following directories to the **Path** environment variable:
   
   * `xxx\opencv\build\x64\vc14\bin`
   * `xxx\opencv\build\x64\vc14\lib`

3. Run Cmake and set the source and build paths:
   
   | Field                       | Path                         |
   | :----                       | :----                        |
   | Where is the source code    | xxx\AcquireProfileData       |
   | Where to build the binaries | xxx\AcquireProfileData\build |

4. Click the **Configure** button. In the pop-up window, set the generator and platform according to the actual situation, and then click the **Finish** button.
5. When the log displays **Configuring done**, click the **Generate** button. When the log displays **Generating done**, click the **Open Project** button.
6. In Visual Studio, change the Solution Configuration from **Debug** to **Release**.
7. Right-click the sample in **Solution Explorer**, and select **Set as Startup Project**.
8. Click **Local Windows Debugger** to build the solution.
9. Enter the index of the camera to which you want to connect, and press the Enter key. The obtained files are saved to the `build` folder.