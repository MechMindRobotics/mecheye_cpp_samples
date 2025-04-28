if (CMAKE_HOST_WIN32)
    cmake_minimum_required(VERSION 3.19)
else ()
    cmake_minimum_required(VERSION 3.5)
endif ()

project(MechEyeCppSamples)

if (CMAKE_HOST_UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif ()

option(USE_OPENCV "Enable samples which depend on OpenCV" ON)

set(SAMPLES
        NoiseRemoval
        BlindSpotFiltering
        ProfileAlignment
        ManageUserSets
        RegisterProfilerEvent
        TriggerMultipleProfilersSimultaneously
        TriggerNonStopAcquisition
        TriggerWithExternalDeviceAndEncoder
        TriggerWithExternalDeviceAndFixedRate
        TriggerWithSoftwareAndEncoder
        TriggerWithSoftwareAndFixedRate
        UseVirtualDevice
        PrintProfilerStatus
        RenderDepthMap
        TransformPointCloud
        MultipleProfilersCalibration
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
)

macro(disable_samples DEPENDENCY_NAME)
    message("${DEPENDENCY_NAME} samples have been disabled:")
    foreach (SAMPLE ${SAMPLES})
        get_filename_component(SAMPLE_NAME ${SAMPLE} NAME)
        set(DEPENDENCY_LIST "${DEPENDENCY_NAME}_DEPENDING")
        if (${SAMPLE_NAME} IN_LIST ${DEPENDENCY_LIST})
            message("  - ${SAMPLE}")
            list(REMOVE_ITEM SAMPLES ${SAMPLE})
        endif ()
    endforeach ()
endmacro()

if (NOT USE_OPENCV)
    disable_samples("OpenCV")
endif ()


message(STATUS "All samples: ${SAMPLES}")

foreach (SAMPLE ${SAMPLES})
    add_subdirectory(${SAMPLE})
endforeach ()
