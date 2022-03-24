========================================================================
    CONSOLE APPLICATION : captureResultToOpenCV Project Overview
========================================================================

This is a simple example of how to find and connect an available Mech-Eye Device
and then convert color and depth data from Mech-Eye Device format to OpenCV format.

You will learn how to:

* use OpenCV in a project with Mech-Eye API,
* convert captured frame into OpenCV format.

How to build:

1. Copy captureResultToOpenCV folder to a location with Read and
   Write permissions (using the name <source>)
2. Install OpenCV 3.4.5 (example works and is tested with version 3.4.5)
3. Edit CMakeLists.txt file to set up paths to your OpenCV folder
   3.1 windows
       - replace "D:/opencv/opencv/build/x64/vc14/lib" with your correct
         path (line 13 set(OpenCV_DIR "D:/opencv/opencv/build/x64/vc14/lib")	
4. Open CMake
   4.1. Set Source code to <source>
   4.2. Set Binaries to <source>/_build or any other writable location
   4.3. Click Configure and Generate
5. Build project
6. Run captureResultToOpenCV
   6.1. Connect to a device
7. Run captureResultToOpenCV application
8. Add your code to run your code from this sample

The application will print out basic info about Mech-Eye Device. 
Save the color and depth data as png file on the disk.

/////////////////////////////////////////////////////////////////////////////
