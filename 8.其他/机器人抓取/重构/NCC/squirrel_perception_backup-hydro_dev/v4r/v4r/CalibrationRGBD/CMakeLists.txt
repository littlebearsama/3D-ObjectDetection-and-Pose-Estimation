
set(V4R_CALIBRATION_RGBD 0 CACHE BOOL "Calibration modules of v4r" )
if(V4R_CALIBRATION_RGBD)

project(v4rCalibrationRGBD)

# OpenCV
find_package(OpenCV REQUIRED)

# OpenNI 2
find_package(PkgConfig REQUIRED)
pkg_search_module(OpenNI2 REQUIRED libopenni2)
include_directories(${OpenNI2_INCLUDE_DIRS})
link_directories(${OpenNI2_LIBRARY_DIRS})


set(SOURCES
        cam.cpp
        dccam.cpp
        dsensor.cpp
)

set(HEADERS
        cam.h
        dccam.h
        dsensor.h
)


add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
v4r_add_library(${PROJECT_NAME} "${HEADERS}")
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${OpenNI2_LIBRARIES})


endif(V4R_CALIBRATION_RGBD)
