
cmake_minimum_required (VERSION 3.10)

project (CaptureSequentiallyMultiCamera)

add_definitions("-fpermissive")
add_definitions("-std=c++11")

set(Files
    ${PROJECT_SOURCE_DIR}/CaptureSequentiallyMultiCamera.cpp
)

if (CMAKE_HOST_WIN32)
    find_package(MechEyeApi REQUIRED CONFIG PATHS "$ENV{MECHEYE_DIR}/API")

    find_package(OpenCV REQUIRED)
    find_package(PCL 1.12.1 REQUIRED)

    include_directories(${MECHEYEAPI_INCLUDE_DIRS}
                        ${OpenCV_INCLUDE_DIRS}
                        ${PCL_INCLUDE_DIRS})
    link_directories(${MECHEYEAPI_LIBRARY_DIRS}
                     ${OpenCV_LIBRARY_DIRS}
                     ${PCL_LIBRARY_DIRS})

    add_executable (${PROJECT_NAME}  ${Files})

    target_link_libraries(${PROJECT_NAME}
                          ${MECHEYEAPI_LIBRARIES}
                          ${OpenCV_LIBS}
                          ${PCL_LIBRARIES}
    )
elseif (CMAKE_HOST_UNIX)

    # OpenCV_DIR : set as your OpenCV libraries directory
    set(OpenCV_DIR "/home/ubuntu/3rdParties/opencv-3.4.5/build")
    find_package(OpenCV REQUIRED)
    find_package(PCL REQUIRED)

    include_directories(${OpenCV_INCLUDE_DIRS}
                                 ${PCL_INCLUDE_DIRS})
    link_directories(${OpenCV_LIBRARY_DIRS}
                            ${PCL_LIBRARY_DIRS})

    include(FindPkgConfig)
    pkg_check_modules(PKGS REQUIRED MechEyeApi)
    include_directories(${PKGS_INCLUDE_DIRS})
    link_directories(${PKGS_LIBRARY_DIRS})

    add_executable (${PROJECT_NAME}  ${Files})

    target_link_libraries(${PROJECT_NAME}
                          ${OpenCV_LIBS}
                          ${PCL_LIBRARIES}
                          MechEyeApi
    )
endif ()
