# CaptureColorMap Project Overview

This is a simple example of how to find and connect an available Mech-Eye Device
and then capture a color map and convert to OpenCV format.

You will learn how to:

* use OpenCV in a project with Mech-Eye API,
* convert captured frame into OpenCV format.

How to build:

* Windows:
  1. Copy CaptureColorMap folder to a location with Read and
   Write permissions (using the name `source`)
  2. Install OpenCV 3.4.5 (example works and is tested with version 3.4.5)
  3. Open CMake
      * Set Source code to `source`
      * Set Binaries to `source`/_build or any other writable location
      * Click Configure and Generate
  4. Open .sln file in Visual Studio
      * Build the solution by `Ctrl + Shift + B`
  5. Run CaptureColorMap
      * Connect to a device
  6. Add your code to run your code from this sample
* Linux:
  1. Edit CMakeLists.txt file to set up paths to your OpenCV folder
  2. Configure the project with

      ```bash
      cmake .
      ```

  3. Build the project with

      ```bash
      make
      ```

  4. Run the project with

      ```bash
      ./CaptureColorMap
      ```

The application will print out basic info about Mech-Eye Device.
Save the color data as png file on the disk.
