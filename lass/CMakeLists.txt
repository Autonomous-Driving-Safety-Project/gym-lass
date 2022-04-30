cmake_minimum_required(VERSION 3.16)

set(TARGET Lass)
project(${TARGET})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_SHARED_LIBRARY_PREFIX "")
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_DIRS} ~/anaconda3/include/python3.8)
find_package(Boost 1.71.0 COMPONENTS python REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(src lib/includes)
link_directories(lib/bin/esmini lib/bin/osi ${Boost_LIBRARY_DIRS})

add_library(${TARGET} SHARED
        src/lass.cpp
        src/utils/utils.cpp
        src/controller/naive.cpp
        src/sensor/ideal_sensor.cpp
        src/sensor/lane_sensor.cpp
        src/sensor/road_sensor.cpp
        )

target_link_libraries(${TARGET}
        boost_python38
        ${PYTHON_LIBRARIES}
        ${Boost_LIBRARIES}
        libesminiLib.so
        libesminiRMLib.so
        libprotobuf.so
        libopen_simulation_interface.so
        )
