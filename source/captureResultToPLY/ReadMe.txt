========================================================================
    CONSOLE APPLICATION : captureResultToPLY Project Overview
========================================================================

This is a simple example of how to find and connect an available Mech-Eye Device
and then convert color and point cloud data from Mech-Eye Device format to PCL format.

You will learn how to:

* use PCL in a project with Mech-Eye API,
* convert captured frame into PCL format.

How to build:

1. Copy captureResultToPLY folder to a location with Read and
   Write permissions (using the name <source>)
2. Install PCL 1.12.1 AllInOne (example works and is tested with version 1.12.1)	
3. Open CMake
   3.1. Set Source code to <source>
   3.2. Set Binaries to <source>/_build or any other writable location
   3.3. Click Configure and Generate
4. Build project
5. Run captureResultToPLY
   5.1. Connect to a device
6. Run captureResultToPLY application
7. Add your code to run your code from this sample

The application will print out basic info about Mech-Eye Device. 
Save the color and point cloud data as ply file on the disk.

/////////////////////////////////////////////////////////////////////////////
