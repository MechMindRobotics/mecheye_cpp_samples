# CaptureHDRPointCloud Project Overview

This is a simple example of how to find and connect to an available Mech-Eye Device
and capture HDR point cloud then convert color and point cloud data from Mech-Eye Device format to PCL format.

You will learn how to:

* use PCL in a project with Mech-Eye API,
* convert captured frame into PCL format.

How to build:

* Windows:
  1. Copy CaptureHDRPointCloud folder to a location with Read and
   Write permissions (using the name `source`)
  2. Install PCL 1.12.1 AllInOne (example works and is tested with version 1.12.1)
  3. Open CMake
        * Set Source code to `source`
        * Set Binaries to `source`/_build or any other writable location
        * Click Configure and Generate
  4. Open .sln file in Visual Studio
        * Build the solution by `Ctrl + Shift + B`
  5. Run CaptureHDRPointCloud
        * Connect to a device
  6. Add your code to run your code from this sample
* Linux:
  1. Configure the project with

      ```bash
      cmake .
      ```

  2. Build the project with

      ```bash
      make
      ```

  3. Run the project with

      ```bash
      ./CaptureHDRPointCloud
      ```

The application capture HDR point cloud.  
Save the color and point cloud data as ply file on the disk.
