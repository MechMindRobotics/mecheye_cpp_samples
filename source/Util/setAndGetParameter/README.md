# SetAndGetParameter Project Overview

This is a simple example of how to find and connect an available Mech-Eye Device
and then get and set parameter data with the Mech-Eye Device.

You will learn how to:

* use Mech-Eye API,
* get and set parameter data.

How to build:

* Windows:
  1. Copy setAndGetParameter folder to a location with Read and
   Write permissions (using the name `source`)
  2. Open CMake
        * Set Source code to `source`
        * Set Binaries to `source`/_build or any other writable location
        * Click Configure and Generate
  3. Open .sln file in Visual Studio
        * Build the solution by `Ctrl + Shift + B`
  4. Run setAndGetParameter
        * Connect to a device
  5. Add your code to run your code from this sample
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
      ./setAndGetParameter
      ```

The application will print out basic info about Mech-Eye Device.
