# CaptureTimedAndPeriodically Project Overview

This is a simple example of how to find and connect an available Mech-Eye Device
and then capture color and depth data from Mech-Eye device timed and periodically.

You will learn how to:

* use Mech-Eye API,
* capture frames from Mech-Eye Device periodically in a specific amount of time.

How to build:

* Windows:
  1. Copy captureTimedAndPeriodically folder to a location with Read and
   Write permissions (using the name `source`)
  2. Install OpenCV 3.4.5 (example works and is tested with version 3.4.5)
  3. Install PCL 1.12.1 AllInOne (example works and is tested with version 1.12.1)
  4. Open CMake
        * Set Source code to `source`
        * Set Binaries to `source`/_build or any other writable location
        * Click Configure and Generate
  5. Open .sln file in Visual Studio
        * Build the solution by `Ctrl + Shift + B`
  6. Run captureTimedAndPeriodically
        * Connect to a device
  7. Add your code to run your code from this sample
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
      ./captureTimedAndPeriodically
      ```

The application will print out basic info about Mech-Eye Device and frame infomation and capture periodically for a specific amount of time.
