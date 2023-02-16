cmake_minimum_required(VERSION 3.5)

project(CapturePointCloudROI)

add_definitions("-std=c++14" "-fpermissive")

set(Files ${PROJECT_SOURCE_DIR}/CapturePointCloudROI.cpp)

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

if(CMAKE_HOST_WIN32)
  find_package(PCL 1.12.1 REQUIRED)
elseif(CMAKE_HOST_UNIX)
  find_package(PCL REQUIRED)
endif()
if(NOT PCL_FOUND)
  message(
    FATAL_ERROR
      "Point Cloud Library (PCL) not found. Please point PCL_DIR to the directory of your PCL installation (containing the file PCLConfig.cmake)."
  )
endif()

include_directories(${MECHEYEAPI_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${MECHEYEAPI_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} ${Files})

target_link_libraries(${PROJECT_NAME} ${MECHEYEAPI_LIBRARIES} ${PCL_LIBRARIES})
