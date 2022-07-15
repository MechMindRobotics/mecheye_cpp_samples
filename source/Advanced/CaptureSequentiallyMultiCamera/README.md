# CaptureSequentiallyMultiCamera Project Overview

This is a simple example of how to find and connect to several available Mech-Eye Devices.  
and convert sequentially captured frame into OpenCV format and PCL format.

You will learn how to:

* capture sequentially using multiple devices,
* use OpenCV in a project with Mech-Eye API,
* use PCL in a porject with Mech-Eye API,
* convert captured frame into OpenCV format,
* convert captured point cloud into PCL format.

How to build:

* Windows:
  1. Copy CaptureSequentiallyMultiCamera folder to a location with Read and
       Write permissions (using the name `source`)
  2. Install OpenCV 3.4.5 (example works and is tested with version 3.4.5)
  3. Install PCL 1.12.1 AllInOne (example works and is tested with version 1.12.1)
  4. Open CMake
      * Set Source code to `source`
      * Set Binaries to `source`/_build or any other writable location
      * Click Configure and Generate
  5. Open .sln file in Visual Studio
      * Build the solution by `Ctrl + Shift + B`
  6. Run CaptureSequentiallyMultiCamera
      * Connect to a device
  7. Add your code to run your code from this sample
* Linux:
  1. Edit CMakeLists.txt file to set up paths to your OpenCV folder
  2. Configure the project with  

      ```bash
      cmake .
      ```

  3. Generate the project with

      ```bash
      make
      ```

  4. Run the application with

      ```bash
      ./CaptureSequentiallyMultiCamera
      ```

The application will print out basic info about Mech-Eye Device.
Save the color and depth data as png file on the disk.
Save the point cloud as ply file on the disk.
