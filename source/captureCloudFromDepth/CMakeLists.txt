cmake_minimum_required (VERSION 3.10)

project (captureCloudFromDepth)

set(Files
    ${PROJECT_SOURCE_DIR}/captureCloudFromDepth.cpp
)

find_package(MechEyeApi REQUIRED CONFIG PATHS "$ENV{MECHEYE_DIR}/API")

find_package(PCL 1.12.1 REQUIRED)

include_directories(${MECHEYEAPI_INCLUDE_DIRS}
					${PCL_INCLUDE_DIRS})

link_directories(${MECHEYEAPI_LIBRARY_DIRS}
				 ${PCL_LIBRARY_DIRS})

add_executable (${PROJECT_NAME}  ${Files})

target_link_libraries(${PROJECT_NAME}
					  ${MECHEYEAPI_LIBRARIES} 
					  ${PCL_LIBRARIES})