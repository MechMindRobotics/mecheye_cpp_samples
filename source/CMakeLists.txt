cmake_minimum_required(VERSION 3.5)

project(MechEyeCppSamples)

add_definitions("-fpermissive")
add_definitions("-std=c++14")

option(USE_OPENCV "Enable samples which depend on OpenCV" ON)
option(USE_PCL "Enable samples which depend on Point Cloud Library (PCL)" ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(SAMPLES
    Basic/ConnectToCamera
    Basic/ConnectAndCaptureImage
    Basic/CaptureColorMap
    Basic/CaptureDepthMap
    Basic/CaptureHDRPointCloud
    Basic/CapturePointCloud
    Basic/CapturePointCloudROI
    Advanced/CaptureCloudFromDepth
    Advanced/CaptureSequentiallyMultiCamera
    Advanced/CaptureSimultaneouslyMultiCamera
    Advanced/CaptureTimedAndPeriodically
    Util/GetCameraIntri
    Util/PrintDeviceInfo
    Util/SetParameters
    Util/SetDepthRange
    Util/SetUserSets
    Laser/SetLaserFramePartitionCount
    Laser/SetLaserFrameRange
    Laser/SetLaserFringeCodingMode
    Laser/SetLaserPowerLevel)

set(PCL_DEPENDING
    CaptureHDRPointCloud
    CapturePointCloud
    CapturePointCloudROI
    CaptureCloudFromDepth
    CaptureSequentiallyMultiCamera
    CaptureSimultaneouslyMultiCamera
    CaptureTimedAndPeriodically)

set(OpenCV_DEPENDING
    CaptureColorMap CaptureDepthMap CaptureSequentiallyMultiCamera
    CaptureSimultaneouslyMultiCamera CaptureTimedAndPeriodically)

macro(disable_samples DEPENDENCY_NAME)
  message("${DEPENDENCY_NAME} samples have been disabled:")
  foreach(SAMPLE ${SAMPLES})
    get_filename_component(SAMPLE_NAME ${SAMPLE} NAME)
    set(DEPENDENCY_LIST "${DEPENDENCY_NAME}_DEPENDING")
    if(${SAMPLE_NAME} IN_LIST ${DEPENDENCY_LIST})
      message("  - ${SAMPLE}")
      list(REMOVE_ITEM SAMPLES ${SAMPLE})
    endif()
  endforeach()
endmacro()

if(CMAKE_HOST_WIN32)
  find_package(MechEyeApi REQUIRED CONFIG PATHS "$ENV{MECHEYE_DIR}/API")

  if(USE_PCL)
    find_package(PCL 1.12.1 REQUIRED)
    if(NOT PCL_FOUND)
      message(
        FATAL_ERROR
          "Point Cloud Library (PCL) not found. Please point PCL_DIR to the directory of your PCL installation (containing the file PCLConfig.cmake), or disable the PCL samples with -DUSE_PCL=OFF."
      )
    endif()
  else()
    disable_samples("PCL")
  endif()

  if(USE_OPENCV)
    find_package(OpenCV REQUIRED)
    if(NOT OpenCV_FOUND)
      message(
        FATAL_ERROR
          "OpenCV not found. Please point OpenCV_DIR to the directory of your OpenCV installation (containing the file OpenCVConfig.cmake), or disable the OpenCV samples  with -DUSE_OPENCV=OFF."
      )
    endif()
  else()
    disable_samples("OpenCV")
  endif()

  message(STATUS "All samples: ${SAMPLES}")
elseif(CMAKE_HOST_UNIX)
  # OpenCV_DIR: set as your OpenCV libraries directory
  set(OpenCV_DIR "/home/ubuntu/3rdParties/opencv-3.4.5/build")

  if(USE_PCL)
    find_package(PCL REQUIRED)
    if(NOT PCL_FOUND)
      message(
        FATAL_ERROR
          "Point Cloud Library (PCL) not found. Please point PCL_DIR to the directory of your PCL installation (containing the file PCLConfig.cmake), or disable the PCL samples with -DUSE_PCL=OFF."
      )
    endif()
  else()
    disable_samples("PCL")
  endif()

  if(USE_OPENCV)
    find_package(OpenCV REQUIRED)
    if(NOT OpenCV_FOUND)
      message(
        FATAL_ERROR
          "OpenCV not found. Please point OpenCV_DIR to the directory of your OpenCV installation (containing the file OpenCVConfig.cmake), or disable the OpenCV samples  with -DUSE_OPENCV=OFF."
      )
    endif()
  else()
    disable_samples("OpenCV")
  endif()

  message(STATUS "All samples: ${SAMPLES}")
endif()

set(SAMPLE_NAMES_LIST_FILE "${CMAKE_BINARY_DIR}/SampleNames.txt")
file(REMOVE ${SAMPLE_NAMES_LIST_FILE})

if(CMAKE_HOST_WIN32)
  foreach(SAMPLE ${SAMPLES})
    get_filename_component(SAMPLE_NAME ${SAMPLE} NAME)

    file(APPEND "${SAMPLE_NAMES_LIST_FILE}" ${SAMPLE}\n)

    include_directories(${MECHEYEAPI_INCLUDE_DIRS})
    link_directories(${MECHEYEAPI_LIBRARY_DIRS})
    add_executable(${SAMPLE_NAME} ${SAMPLE}/${SAMPLE_NAME}.cpp)

    target_link_libraries(${SAMPLE_NAME} ${MECHEYEAPI_LIBRARIES})

    if(${SAMPLE_NAME} IN_LIST PCL_DEPENDING)
      include_directories(${PCL_INCLUDE_DIRS})
      link_directories(${PCL_LIBRARY_DIRS})
      target_link_libraries(${SAMPLE_NAME} ${PCL_LIBRARIES})
    endif()

    if(${SAMPLE_NAME} IN_LIST OpenCV_DEPENDING)
      include_directories(${OpenCV_INCLUDE_DIRS})
      link_directories(${OPENCV_LIBRARY_DIRS})
      target_link_libraries(${SAMPLE_NAME} ${OpenCV_LIBS})
    endif()
  endforeach()
elseif(CMAKE_HOST_UNIX)
  foreach(SAMPLE ${SAMPLES})
    get_filename_component(SAMPLE_NAME ${SAMPLE} NAME)

    file(APPEND "${SAMPLE_NAMES_LIST_FILE}" ${SAMPLE}\n)

    include(FindPkgConfig)
    pkg_check_modules(PKGS REQUIRED MechEyeApi)
    include_directories(${PKGS_INCLUDE_DIRS})
    link_directories(${PKGS_LIBRARY_DIRS})

    add_executable(${SAMPLE_NAME} ${SAMPLE}/${SAMPLE_NAME}.cpp)

    target_link_libraries(${SAMPLE_NAME} MechEyeApi)

    if(${SAMPLE_NAME} IN_LIST PCL_DEPENDING)
      include_directories(${PCL_INCLUDE_DIRS})
      link_directories(${PCL_LIBRARY_DIRS})
      target_link_libraries(${SAMPLE_NAME} ${PCL_LIBRARIES})
    endif()

    if(${SAMPLE_NAME} IN_LIST OpenCV_DEPENDING)
      include_directories(${OpenCV_INCLUDE_DIRS})
      link_directories(${OPENCV_LIBRARY_DIRS})
      target_link_libraries(${SAMPLE_NAME} ${OpenCV_LIBS})
    endif()
  endforeach()
endif()
