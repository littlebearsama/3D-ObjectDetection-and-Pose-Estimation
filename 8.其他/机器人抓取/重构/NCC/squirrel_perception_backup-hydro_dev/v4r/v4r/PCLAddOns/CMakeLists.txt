SET(V4R_PCL_ADD_ONS 1 CACHE BOOL "PCL functions, utils and specific applications." )
IF(V4R_PCL_ADD_ONS)

#FIND_PACKAGE (OpenCV QUIET)
#include_directories(${OPENCV_INCLUDE_DIRS})
#link_directories(${OPENCV_LIBRARY_DIRS})

IF(${PCL_FOUND})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

  SET(CMAKE_BUILD_TYPE Release)

  PROJECT(v4rPCLAddOns)

  SET(SOURCE_CPP
    BilateralFilter.cc
    CCEuclideanClustering.cc
    CCLabeling.cc
    ContourDetection.cc
    EuclideanClustering.cc
    HoleFillingBF.cc
    ModelFitter.cpp
    NormalsClustering.cc
    NormalsEstimationNR.cc
    PCLFunctions.cpp
    PCLUtils.cpp
    PlanePopout.cc
    SubsamplePointCloud.cc
  )
  SET(SOURCE_H
    BilateralFilter.hh
    CCEuclideanClustering.hh
    CCLabeling.hh
    ContourDetection.hh
    EuclideanClustering.hh
    HoleFillingBF.hh
    ModelFitter.h
    NormalsClustering.hh
    NormalsEstimationNR.hh
    PCLFunctions.h
    PCLUtils.h
    PCLCommonHeaders.h
    PlanePopout.hh
    SubsamplePointCloud.hh
  )

  add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
  target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
  v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ELSE (${PCL_FOUND})
  MESSAGE("\nERROR: PCLAddOns: Could not locate PCL. Skip PCL_ADD_ONS\n")
ENDIF(${PCL_FOUND})
ENDIF(V4R_PCL_ADD_ONS)

