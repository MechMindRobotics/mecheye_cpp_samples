# MultipleProfilersCalibration Sample

With this sample, you can calibrate multiple profilers that simultaneously scan the same target object, and output the calibration results and errors, stitching results, the stitched depth map, and the stitched point cloud.

If you have any questions or have anything to share, feel free to post on the [Mech-Mind Online Community](https://community.mech-mind.com/). The community also contains a [specific category for development with Mech-Eye SDK](https://community.mech-mind.com/c/mech-eye-sdk-development/19).

## Build the Sample

Prerequisites and instructions for building the sample on Windows and Ubuntu are provided.

### Windows

#### Prerequisites

The following software are required to build this sample. Please download and install these software.

* [Mech-Eye SDK (latest version)](https://downloads.mech-mind.com/?tab=tab-sdk)
* [Visual Studio (version 2017 or above)](https://visualstudio.microsoft.com/vs/community/)
* [CMake (version 3.2 or above)](https://cmake.org/download/)
* [OpenCV (version 3.4.5 or above)](https://opencv.org/releases/)

#### Instructions

1. Make sure that the sample is stored in a location with read and write permissions.
2. Add the following directories to the **Path** environment variable:

   * `xxx/opencv/build/x64/vc14/bin`
   * `xxx/opencv/build/x64/vc14/lib`

3. Run Cmake and set the source and build paths:

   | Field                       | Path                                   |
   | :-------------------------- | :------------------------------------- |
   | Where is the source code    | xxx/MultipleProfilersCalibration       |
   | Where to build the binaries | xxx/MultipleProfilersCalibration/build |

4. Click the **Configure** button. In the pop-up window, set the generator and platform according to the actual situation, and then click the **Finish** button.
5. When the log displays **Configuring done**, click the **Generate** button. When the log displays **Generating done**, click the **Open Project** button.
6. In Visual Studio toolbar, change the solution configuration from **Debug** to **Release**.
7. In the **Solution Explorer** panel, right-click the sample, and select **Set as Startup Project**.
8. Click the **Local Windows Debugger** button in the toolbar to run the sample.
9. Enter the index of the laser profiler to which you want to connect, and press the Enter key. The obtained files are saved to the `build` folder.

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

* Install [Mech-Eye SDK (latest version)](https://downloads.mech-mind.com/?tab=tab-sdk).

  >Note: If you have installed Mech-Eye SDK before, please uninstall it first with the following command:
  >
  >```bash
  >sudo dpkg -P MechEyeApi
  >```
  
  * If the system architecture is AMD64, execute the following command:

    ```bash
    sudo dpkg -i 'Mech-Eye_API_x.x.x_amd64.deb'
    ```

  * If the system architecture is ARM64, execute the following command:

    ```bash
    sudo dpkg -i 'Mech-Eye_API_x.x.x_arm64.deb'
    ```

* Install third-party libraries: OpenCV is required.

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

#### Instructions

1. Navigate to the directory of the sample.

   ```bash
   cd xxx/profiler/Calibration/MultipleProfilersCalibration/
   ```

2. Configure and build the sample.

   ```bash
   sudo mkdir build && cd build
   sudo cmake ..
   sudo make
   ```

3. Run the sample.

   ```bash
   sudo ./MultipleProfilersCalibration
   ```

4. Enter the index of the laser profiler to which you want to connect, and press the Enter key. The obtained files are saved to `/MultipleProfilersCalibration/build`.

## Sample Usage

### Overview

This sample program performs calibration and stitching operations for multiple laser profilers that scan the same object. It supports:

1. Calibration of laser profilers relative to a primary device.
2. Point cloud stitching in the scenario of one primary device.
3. Point cloud stitching in the scenario of multiple primary devices.

---

### 1. Main Menu Operations

Launch the program and choose an operation:

| Command | Description                                                        |
| ------- | ------------------------------------------------------------------ |
| `C`     | Calibrate secondary profilers relative to a primary laser profiler |
| `S`     | Stitch point clouds using one primary laser profiler               |
| `M`     | Stitch point clouds using multiple primary laser profilers         |

---

### 2. Calibration Workflow (Option: `C`)

#### Step 1: Define Frustum Dimensions

Input the dimensions of the frustums of the calibration target when prompted:

* **Top Length** (mm): Upper base length of the frustums (e.g., `100.0`).
* **Bottom Length** (mm): Lower base length of the frustums (e.g., `150.0`).
* **Height** (mm): Height of the frustums (e.g., `50.0`).

> 💡 *Confirm values with `y` or re-enter if incorrect.*

#### Step 2: Connect Profilers

* Automatic detection connects all available profilers.
* Ensure devices are powered and accessible via SDK.

#### Step 3: Identify Devices

* **Major Profiler**: Primary reference device (usually first in the device list)
* **Minor Profilers**: Secondary devices to calibrate relative to the primary device.

#### Step 4: Set Calibration Parameters

For each secondary laser profiler, configure:

**Mode Selection:**

* `wide` (translation-based, i.e., side-by-side or in the reversed direction)
* `angle` (rotation-based, i.e., in the opposite direction or around a circle)

**Pose Parameters:**

* *Translation Mode*:
  * Distance (mm) between two frustums of the calibration target (e.g., `200.0`).
  * Translation axis (`X`, `Y`, or `Z`): the axis along which one frustum is translated relative to another.
* *Rotation Mode*:
  * Rotation angle (°) between two frustums of the calibration target (e.g., `30.0`).
  * Rotation radius (mm): distance from the rotation center to the midpoint of the frustum's height (e.g., `50.0`).
  * Rotation axis (`X`, `Y`, or `Z`): the axis along which one frustum is rotated relative to another.

> 💡 *Review inputs and confirm with `y`.*

#### Step 5: Execute Calibration

* Data acquisition is initiated automatically.
* **Ensure**:
  * A clear view of the calibration target
  * Stable ambient lighting conditions
* Results are saved automatically to the specified folder (e.g., `C:/calibration_results`)

#### Step 6: Stitching (Optional)

* After calibration, enter `y` to start stitching and/or fusing:
  * Stitched point cloud is saved as a `.ply` file.
  * Perform fusion if the laser profilers are Z-parallel and not in the opposite setup.

---

### 3. Single Primary Stitching (Option: `S`)

#### Step 1: Connect Laser Profilers

* Devices are automatically detected.

#### Step 2: Acquire Data

* System acquires 2D images and depth maps from all profilers.

#### Step 3: Load Calibration Data

* Provide path to existing calibration files (e.g., `C:/calibration_files`)

#### Step 4: Stitch and Save

* Stitched point cloud is generated and saved at specified path.
* Follow on-screen prompts for fusion (if applicable).

---

### 4. Multi-Primary Stitching (Option: `M`)

#### Step 1: Load Multiple Calibration Files

Input paths for three calibration pairs:

1. Primary 0 + Secondary 1 (e.g., `C:/calib_01`)
2. Primary 0 + Secondary 2 (e.g., `C:/calib_02`)
3. Primary 2 + Secondary 3 (e.g., `C:/calib_23`)

#### Step 2: Connect Profilers

* System verifies connection for all laser profilers.

#### Step 3: Acquire Data

* Data is automatically acquired from all laser profilers.

#### Step 4: Stitch Across Devices

* Computes transformations using loaded calibration data.
* Saves stitched point cloud to user-defined path.

---

### Key Notes

#### File Paths

* Use **English-only characters** in paths (e.g., avoid characters in `中文` or `日本語`)
* Ensure write permissions to target folders

#### Profiler Arrangement

* Z-parallel setups yield best fusion results.
* Opposite setups may prevent fusion.

#### Error Handling

* Detailed error messages are displayed for failed data acquisition or calibrations.
* Check device connections and retry.

### Troubleshooting Tips

| Error Message          | Solution                                          |
| ---------------------- | ------------------------------------------------- |
| "Capture failed"       | Ensure all laser profilers are visible to system  |
| "Failed to save files" | Verify folder permissions and path validity       |
| Calibration errors     | Re-measure target dimensions and retry            |
