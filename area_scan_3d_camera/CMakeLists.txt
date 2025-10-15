if(CMAKE_HOST_WIN32 AND USE_PCL)
  cmake_minimum_required(VERSION 3.19)
else()
  cmake_minimum_required(VERSION 3.5)
endif()

project(MechEyeCppSamples)

if(CMAKE_HOST_UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

option(USE_OPENCV "Enable samples which depend on OpenCV" ON)
option(USE_PCL "Enable samples which depend on Point Cloud Library (PCL)" ON)
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(USE_HALCON FALSE)
  add_compile_options(-mno-outline-atomics)
else()
  option(USE_HALCON "Enable samples which depend on Halcon" ON)
endif()

set(SAMPLES
    Basic/ConnectToCamera
    Basic/ConnectAndCaptureImages
    Basic/Capture2DImage
    Basic/CaptureDepthMap
    Basic/CapturePointCloudHDR
    Basic/CapturePointCloud
    Basic/CapturePointCloudWithNormals
    Basic/SaveVirtualDevice
    Advanced/CaptureStereo2DImages
    Advanced/RenderDepthMap
    Advanced/ConvertDepthMapToPointCloud
    Advanced/MultipleCamerasCaptureSequentially
    Advanced/MultipleCamerasCaptureSimultaneously
    Advanced/CapturePeriodically
    Advanced/Mapping2DImageToDepthMap
    Advanced/RegisterCameraEvent
    Advanced/SetParametersOfLaserCameras
    Advanced/SetParametersOfUHPCameras
    Advanced/TransformPointCloud
    Advanced/WarmUp
    Util/GetCameraIntrinsics
    Util/PrintCameraInfo
    Util/SetPointCloudProcessingParameters
    Util/SetScanningParameters
    Util/SetDepthRange
    Util/ManageUserSets
    Util/SaveAndLoadUserSet
    Calibration/HandEyeCalibration
    Halcon/ConvertPointCloudToObjectModel3D
    Pcl/ConvertPointCloudToPcl
    Pcl/ConvertPointCloudWithNormalsToPcl
    )

set(PCL_DEPENDING
   ConvertPointCloudToPcl
   ConvertPointCloudWithNormalsToPcl
)

set(OpenCV_DEPENDING
    Capture2DImage
    CaptureDepthMap
    RenderDepthMap
    CaptureStereo2DImages
    MultipleCamerasCaptureSequentially
    MultipleCamerasCaptureSimultaneously
    CapturePeriodically
    HandEyeCalibration
    )

set(Halcon_DEPENDING ConvertPointCloudToObjectModel3D)

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


if(NOT USE_OPENCV)
  disable_samples("OpenCV")
endif()

if(NOT USE_PCL)
  disable_samples("PCL")
endif()

if(NOT USE_HALCON)
  disable_samples("Halcon")
endif()

message(STATUS "All samples: ${SAMPLES}")

foreach(SAMPLE ${SAMPLES})
  add_subdirectory(${SAMPLE})
endforeach()
