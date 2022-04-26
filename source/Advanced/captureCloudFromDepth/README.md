# CaptureCloudFromDepth Project Overview

This is a simple example of how to find and connect an available Mech-Eye Device and then construct color and point cloud data from depth map and color map.

You will learn how to:

* use PCL in a project with Mech-Eye API,
* convert captured frame into PCL format,
* construct point cloud from depth map.

How to build:

* Windows:
   1. Copy captureCloudFromDepth folder to a location with Read and
      Write permissions (using the name `source`)
   2. Install PCL 1.12.1 AllInOne (example works and is tested with version 1.12.1)
   3. Open CMake
      * Set Source code to `source`
      * Set Binaries to `source`/_build or any other writable location
      * Click Configure and Generate
   4. Open .sln file in Visual Studio
      * Build the solution by `Ctrl + Shift + B`
   5. Run captureCloudFromDepth
      * Connect to a device
   6. Add your code to run your code from this sample
* Linux:
   1. Configure the project by

      ```bash
      cmake .
      ```

   2. Generate the project by

      ```bash
      make
      ```

   3. Run the application by

      ```bash
      ./captureCloudFromDepth
      ```

The application will print out basic info about Mech-Eye Device.
Save the color and point cloud data as ply file on the disk.
