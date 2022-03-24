
cmake_minimum_required (VERSION 3.10)

project (captureResultToOpenCV)

set(Files
    ${PROJECT_SOURCE_DIR}/captureResultToOpenCV.cpp
)

find_package(MechEyeApi REQUIRED CONFIG PATHS "$ENV{MECHEYE_DIR}/API")

# OpenCV_DIR : set as your OpenCV libraries directory
set(OpenCV_DIR "D:/opencv/opencv/build/x64/vc14/lib")
find_package(OpenCV REQUIRED)

include_directories(${MECHEYEAPI_INCLUDE_DIRS}
					${OpenCV_INCLUDE_DIRS})
link_directories(${MECHEYEAPI_LIBRARY_DIRS}
				${OpenCV_LIBRARY_DIRS})

add_executable (${PROJECT_NAME}  ${Files})

target_link_libraries(${PROJECT_NAME}
    ${MECHEYEAPI_LIBRARIES} 
	${OpenCV_LIBS}
)
