SET(V4R_KEYPOINT_CONVERTIONS 1 CACHE BOOL "Convertions of pcl/ opencv/ keypoint data types" )
IF(V4R_KEYPOINT_CONVERTIONS)

#FIND_PACKAGE (OpenCV)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})
#include_directories("/usr/include/eigen3")

find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

PROJECT(v4rKeypointConvertions)
SET(SOURCE_CPP
)

SET(SOURCE_H
  convertImage.hpp
  convertCloudKP.hpp
)

#add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
#target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rKeypointTools)
#v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_KEYPOINT_CONVERTIONS)

