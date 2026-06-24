# MoveDirVecCalibration Sample

Using this sample, you can complete the calibration of the movement direction vector for a single laser profiler, obtain the calibrated movement direction vector, the corresponding reprojection error, and correction parameters for downstream processing (including depth maps and pixel offsets obtained from direction correction). The sample aims for engineering reproducibility and result acceptability, demonstrating a complete closed-loop process from acquisition to correction and back to the camera coordinate system.

If you have any questions or have anything to share, feel free to post on the [Mech-Mind Online Community](https://community.mech-mind.com/). The community also contains a [specific category for development with Mech-Eye SDK](https://community.mech-mind.com/c/mech-eye-sdk-development/19).

## Build the Sample

Prerequisites and instructions for building the sample on Windows and Ubuntu are provided.

### Windows

#### Prerequisites

The following software are required to build this sample. Please download and install these software.

- [Mech-Eye SDK (latest version)](https://downloads.mech-mind.com/?tab=tab-sdk)
- [Visual Studio (version 2017 or above)](https://visualstudio.microsoft.com/vs/community/)
- [CMake (version 3.2 or above)](https://cmake.org/download/)
- [OpenCV (version 3.4.5 or above)](https://opencv.org/releases/)

#### Instructions

1. Make sure that the sample is stored in a location with read and write permissions.
2. Add the following directories to the **Path** environment variable:

   - `xxx/opencv/build/x64/vc14/bin`
   - `xxx/opencv/build/x64/vc14/lib`

3. Run Cmake and set the source and build paths:

   | Field                       | Path                            |
   | :-------------------------- | :------------------------------ |
   | Where is the source code    | xxx/MoveDirVecCalibration       |
   | Where to build the binaries | xxx/MoveDirVecCalibration/build |

4. Click the **Configure** button. In the pop-up window, set the generator and platform according to the actual situation, and then click the **Finish** button.
5. When the log displays **Configuring done**, click the **Generate** button. When the log displays **Generating done**, click the **Open Project** button.
6. In Visual Studio toolbar, change the solution configuration from **Debug** to **Release**.
7. In the **Solution Explorer** panel, right-click the sample, and select **Set as Startup Project**.
8. Click the **Local Windows Debugger** button in the toolbar to run the sample.
9. During runtime, enter the index of the laser profiler you want to connect to as prompted. The program will save the generated result files to the build folder (the sample saves three types of files: original depth, corrected depth with offset, and aligned depth (returned to camera coordinate system) for verifying the geometric meaning of each step).

### Ubuntu

Ubuntu 18 or above is required.

#### Prerequisites

- Update the software source list.

  ```bash
  sudo apt-get update
  ```

- Install required tools.

  ```bash
  sudo apt-get install -y build-essential pkg-config cmake
  ```

- Install [Mech-Eye SDK (latest version)](https://downloads.mech-mind.com/?tab=tab-sdk).

  > Note: If you have installed Mech-Eye SDK before, please uninstall it first with the following command:
  >
  > ```bash
  > sudo dpkg -P MechEyeApi
  > ```

- If the system architecture is AMD64, execute the following command:

  ```bash
  sudo dpkg -i 'Mech-Eye_API_x.x.x_amd64.deb'
  ```

- If the system architecture is ARM64, execute the following command:

  ```bash
  sudo dpkg -i 'Mech-Eye_API_x.x.x_arm64.deb'
  ```

- Install third-party libraries: OpenCV is required.

  - Install OpenCV (latest version):

    ```bash
    sudo apt update && sudo apt install -y unzip
    wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
    unzip opencv.zip
    mkdir build && cd build
    cmake ../opencv-4.x
    cmake --build .
    sudo make install
    ```

  > Note: On Ubuntu, OpenCV 4.x is recommended. Ensure that CMake can find OpenCVConfig.cmake during the build process, or specify the path using -DOpenCV_DIR=....

#### Instructions

1. Navigate to the directory of the sample.

   ```bash
   cd xxx/profiler/Calibration/MoveDirVecCalibration
   ```

2. Configure and build the sample.

   ```bash
   sudo mkdir build && cd build
   sudo cmake ..
   sudo make
   ```

3. Run the sample.

   ```bash
   sudo ./MoveDirVecCalibration
   ```

4. Enter the index of the profiler to connect to as prompted and press Enter. The program will save the generated results to `/MoveDirVecCalibration/build` (or the current build directory), including the original depth, direction correction results (with pixel offset), and aligned depth map for stitching, facilitating step-by-step verification..

## Sample Usage

### Overview

This sample demonstrates the complete calibration and correction process for a single profiler moving along a specified direction. The calibration results can be used for subsequent stitching or measurement. The key steps are as follows (the sample saves the results of each step as independent files for verification and troubleshooting):

1. Input calibration target dimensions and pose parameters (to reconstruct geometry and improve calibration stability).
2. Connect to and identify the target profiler device.
3. Capture depth (and optionally intensity) images: at least two frames are required for calibration (A->B and C->D).
4. Perform single-direction movement vector calibration to obtain moveDirVec and reprojection error.
5. Use moveDirVec to perform direction correction (rotation/resampling) on the captured depth maps, resulting in correctedDepth and returning pixelBiasToCamCoord (pixel offset).
6. Use the returned pixel offset to translate correctedDepth back to the original camera coordinate system (removing the offset), resulting in the final alignedDepth which can be directly used for stitching/measurement.
7. Save and output three types of files (as named in the sample) for comparative verification:

- 01_raw_depth.tiff: Original captured depth (scan coordinate system)
- 02_corrected_depth_with_offset.tiff: Direction-corrected (with offset)
- 03_aligned_depth_for_stitching.tiff: Offset-removed, aligned to camera coordinate system (ready for stitching)

---

### 1. Input Target Dimensions

Input the calibration target frustum parameters:

- **Top Length** (mm): Upper base length of the frustums (e.g., `100.0`).
- **Bottom Length** (mm): Lower base length of the frustums (e.g., `150.0`).
- **Height** (mm): Height of the frustums (e.g., `50.0`).
  These parameters are used to reconstruct the target geometry during calibration and constrain point cloud matching, improving calibration robustness.

---

### 2. Connect Profiler

- The program automatically detects and lists visible profiler devices.
- Ensure the device is powered on, connected, and within the visible range.
- Enter the index of the device to connect to as prompted and press Enter.

### 3. Input Target Pose Parameters

For the single profiler:

- Translation Mode:
  - Distance between two target frustums (mm)
  - Translation axis: X, Y, or Z
- Rotation Mode:

  - Rotation angle (°)
  - Rotation radius (mm)
  - Rotation axis: X, Y, or Z

Select the appropriate mode based on the actual calibration stage movement type and input the parameters.

---

### 4. Capture Images

- The sample automatically captures depth and intensity maps.
- Capture requirements: The target must be clearly visible in the field of view, and the start/end positions of the movement segment must match the documentation (to ensure consistent movement between the two captured frames).
- The two captured depth frames serve as input for calibration: depth1 (A->B) and depth2 (C->D).

---

### 5. Perform Move Direction Calibration

- Calculate the actual movement direction vector moveDirVec of the profiler (unit vector in the camera coordinate system) using the two depth maps.
- Output the reprojection errors of the two calibrations (to determine calibration quality).
- It is strongly recommended to first calibrate and lock Y (the main scanning direction) before estimating the X/Z components to improve X/Z calibration accuracy (see comments for details).

---

### 6. Correct Depth Map

- Use moveDirVec to perform direction correction on the captured depth maps (mapping the scan coordinate system to the ideal camera coordinate system).
- The correction process involves rotation and resampling to align the scan line direction with the camera Y-axis.
- Correction outputs include:
  - Corrected depth map (corrected.depth, with direction corrected but origin possibly translated)
  - Pixel offset pixelBiasToCamCoord (in pixels, representing the translation of the corrected image relative to the original camera coordinate system origin)
  - (In the sample) corrected.depth needs to be translated based on the pixel offset to return to the camera coordinate system, resulting in alignedDepth, which can be directly used for stitching or measurement.

---

### Key Notes

#### File Paths

- Use **English-only characters** in paths (e.g., avoid characters in `中文` or `日本語`)
- Ensure write permissions to target folders

#### Error Handling

- The sample prints detailed error information. If data acquisition or calibration fails, check the device connection, target placement, and acquisition parameters as prompted and try again.
- Common failure reasons include: target occlusion, inconsistent movement between captured frames, poor alignment between camera and target, etc.

### Troubleshooting Tips

| Error Message          | Solution                                         |
| ---------------------- | ------------------------------------------------ |
| "Capture failed"       | Ensure all laser profilers are visible to system |
| "Failed to save files" | Verify folder permissions and path validity      |
| Calibration errors     | Re-measure target dimensions and retry           |
