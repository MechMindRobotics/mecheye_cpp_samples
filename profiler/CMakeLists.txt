if(CMAKE_HOST_WIN32)
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
    add_compile_options(-mno-outline-atomics)
endif()

set(SAMPLES
    Advanced/NoiseRemoval
    Advanced/BlindSpotFiltering
    Advanced/ProfileAlignment
    Advanced/RegisterProfilerEvent
    Advanced/TriggerMultipleProfilersSimultaneously
    Advanced/TransformPointCloud
    Advanced/UseVirtualDevice
    Advanced/WarmUp
    Basic/TriggerNonStopAcquisition
    Basic/TriggerWithExternalDeviceAndEncoder
    Basic/TriggerWithExternalDeviceAndFixedRate
    Basic/TriggerWithSoftwareAndEncoder
    Basic/TriggerWithSoftwareAndFixedRate
    Calibration/MultipleProfilersCalibration
    Util/ManageUserSets
    Util/PrintProfilerStatus
    Util/RenderDepthMap
    Util/HandleNanAndNegativeInDepth
    Pcl/ConvertPointCloudToPcl
)

set(PCL_DEPENDING
    ConvertPointCloudToPcl
)

set(OpenCV_DEPENDING
    TriggerMultipleProfilersSimultaneously
    TriggerNonStopAcquisition
    TriggerWithExternalDeviceAndEncoder
    TriggerWithExternalDeviceAndFixedRate
    TriggerWithSoftwareAndEncoder
    TriggerWithSoftwareAndFixedRate
    UseVirtualDevice
    ProfileAlignment
    BlindSpotFiltering
    RenderDepthMap
    NoiseRemoval
    MultipleProfilersCalibration
    HandleNanAndNegativeInDepth
)

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

message(STATUS "All samples: ${SAMPLES}")

foreach(SAMPLE ${SAMPLES})
    add_subdirectory(${SAMPLE})
endforeach()
