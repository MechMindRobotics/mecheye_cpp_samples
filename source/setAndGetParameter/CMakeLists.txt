
cmake_minimum_required (VERSION 3.10)

project (setAndGetParameter)

set(Files
    ${PROJECT_SOURCE_DIR}/setAndGetParameter.cpp
)

find_package(MechEyeApi REQUIRED CONFIG PATHS "$ENV{MECHEYE_DIR}/API")

include_directories(${MECHEYEAPI_INCLUDE_DIRS})
link_directories(${MECHEYEAPI_LIBRARY_DIRS})

add_executable (${PROJECT_NAME}  ${Files})
target_link_libraries(${PROJECT_NAME} ${MECHEYEAPI_LIBRARIES})


