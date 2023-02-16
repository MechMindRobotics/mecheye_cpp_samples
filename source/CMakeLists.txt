cmake_minimum_required(VERSION 3.5)

project(MechEyeCppSamples)

add_definitions("-fpermissive")
add_definitions("-std=c++14")

option(USE_OPENCV "Enable samples which depend on OpenCV" ON)
option(USE_PCL "Enable samples which depend on Point Cloud Library (PCL)" ON)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(USE_HALCON FALSE)
else()
  option(USE_HALCON "Enable samples which depend on Halcon" ON)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(SAMPLES
    Basic/ConnectToCamera
    Basic/ConnectAndCaptureImage
    Basic/CaptureColorMap
    Basic/CaptureDepthMap
    Basic/CaptureHDRPointCloud
    Basic/CapturePointCloud
    Basic/CapturePointCloudROI
    Basic/CapturePointCloudFromTextureMask
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
    Laser/SetLaserPowerLevel
    UHP/SetUHPCaptureMode
    UHP/SetUHPFringeCodingMode
    Halcon/CaptureHalconPointCloud)

set(PCL_DEPENDING
    CaptureHDRPointCloud
    CapturePointCloud
    CapturePointCloudROI
    CaptureCloudFromDepth
    CaptureSequentiallyMultiCamera
    CaptureSimultaneouslyMultiCamera
    CaptureTimedAndPeriodically
    CapturePointCloudFromTextureMask)

set(OpenCV_DEPENDING
    CaptureColorMap CaptureDepthMap CaptureSequentiallyMultiCamera
    CaptureSimultaneouslyMultiCamera CaptureTimedAndPeriodically)

set(Halcon_DEPENDING CaptureHalconPointCloud)

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
  if(NOT MechEyeApi_FOUND)
      message(
      FATAL_ERROR "MechEyeApi not found. Please install MechEyeApi first.")
    endif()
elseif(CMAKE_HOST_UNIX)
  find_package(PkgConfig)
  if(NOT PkgConfig_FOUND)
    message(FATAL_ERROR "PkgConfig not found.")
  else()
    pkg_check_modules(MECHEYEAPI REQUIRED MechEyeApi)
    if(NOT MECHEYEAPI_FOUND)
      message(
        FATAL_ERROR "MechEyeApi not found. Please install MechEyeApi first.")
    endif()
  endif()
  endif()

  if(USE_OPENCV)
  # OpenCV_DIR: set as your OpenCV libraries directory; Uncomment next line to
  # set OpenCV_DIR manually

  # set(OpenCV_DIR  "path to OpenCV directory")
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

if(USE_PCL)
  if(CMAKE_HOST_WIN32)
    find_package(PCL 1.12.1 REQUIRED)
elseif(CMAKE_HOST_UNIX)
    find_package(PCL REQUIRED)
  endif()
    if(NOT PCL_FOUND)
      message(
        FATAL_ERROR
          "Point Cloud Library (PCL) not found. Please point PCL_DIR to the directory of your PCL installation (containing the file PCLConfig.cmake), or disable the PCL samples with -DUSE_PCL=OFF."
      )
    endif()
  else()
    disable_samples("PCL")
  endif()

if(USE_HALCON)
  set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/Halcon/cmake;${CMAKE_MODULE_PATH}")
  find_package(Halcon REQUIRED)
  find_package(HDevEngine REQUIRED)
  if(NOT HALCON_FOUND)
    message(FATAL_ERROR "Halcon not found.")
    endif()
  else()
  disable_samples("Halcon")
  endif()

  message(STATUS "All samples: ${SAMPLES}")

  foreach(SAMPLE ${SAMPLES})
    get_filename_component(SAMPLE_NAME ${SAMPLE} NAME)

    include_directories(${MECHEYEAPI_INCLUDE_DIRS})
    link_directories(${MECHEYEAPI_LIBRARY_DIRS})

  set(EXCUTABLE_LIST ${SAMPLE}/${SAMPLE_NAME}.cpp)
  set(TARGETLINK_LIST ${MECHEYEAPI_LIBRARIES})

    if(${SAMPLE_NAME} IN_LIST PCL_DEPENDING)
      include_directories(${PCL_INCLUDE_DIRS})
      link_directories(${PCL_LIBRARY_DIRS})
    list(APPEND TARGETLINK_LIST ${PCL_LIBRARIES})
    endif()

    if(${SAMPLE_NAME} IN_LIST OpenCV_DEPENDING)
      include_directories(${OpenCV_INCLUDE_DIRS})
      link_directories(${OPENCV_LIBRARY_DIRS})
    list(APPEND TARGETLINK_LIST ${OpenCV_LIBS})
    endif()

  if(${SAMPLE_NAME} IN_LIST Halcon_DEPENDING)
    include_directories(${HALCON_INC_DIRS})
    link_directories(${HALCON_EXT_LIB_DIR})
    list(APPEND TARGETLINK_LIST ${HALCON_LIBS} ${HALCON_LIBS_XL})
    list(APPEND EXCUTABLE_LIST ${SAMPLE}/HalconUtil.cpp)
  endif()

  add_executable(${SAMPLE_NAME} ${EXCUTABLE_LIST})
  target_link_libraries(${SAMPLE_NAME} ${TARGETLINK_LIST})

  endforeach()
