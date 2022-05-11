
cmake_minimum_required (VERSION 3.5)

project (CaptureCloudFromDepth)

add_definitions("-fpermissive")
add_definitions("-std=c++14")

set(Files
    ${PROJECT_SOURCE_DIR}/CaptureCloudFromDepth.cpp
)

if (CMAKE_HOST_WIN32)
    find_package(MechEyeApi REQUIRED CONFIG PATHS "$ENV{MECHEYE_DIR}/API")

    find_package(PCL 1.12.1 REQUIRED)

    include_directories(${MECHEYEAPI_INCLUDE_DIRS}
                        ${PCL_INCLUDE_DIRS})
    link_directories(${MECHEYEAPI_LIBRARY_DIRS}
                     ${PCL_LIBRARY_DIRS})

    add_executable (${PROJECT_NAME}  ${Files})

    target_link_libraries(${PROJECT_NAME}
                          ${MECHEYEAPI_LIBRARIES}
                          ${PCL_LIBRARIES}
    )
elseif (CMAKE_HOST_UNIX)
    find_package(PCL REQUIRED)

    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})

    include(FindPkgConfig)
    pkg_check_modules(PKGS REQUIRED MechEyeApi)
    include_directories(${PKGS_INCLUDE_DIRS})
    link_directories(${PKGS_LIBRARY_DIRS})

    add_executable (${PROJECT_NAME}  ${Files})

    target_link_libraries(${PROJECT_NAME}
                          ${PCL_LIBRARIES}
                          MechEyeApi
    )
endif ()
