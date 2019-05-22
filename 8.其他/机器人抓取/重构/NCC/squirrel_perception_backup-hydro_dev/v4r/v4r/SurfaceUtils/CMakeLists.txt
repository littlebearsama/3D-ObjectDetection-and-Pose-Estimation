SET(V4R_SURFACE_UTILS 1 CACHE BOOL "Utils for surface abstraction." )
IF(V4R_SURFACE_UTILS)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

IF(V4R_TOMGINE)
  add_definitions(-DV4R_TOMGINE)
ENDIF(V4R_TOMGINE)

IF(V4R_PCL_ADD_ONS)
IF(V4R_EPUTILS)

PROJECT(v4rSurfaceUtils)

IF(${PCL_FOUND})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  SET(SOURCE_CPP
    AddGroundTruth.cpp
    BoundaryDetector.cpp
    ContourDetector.cpp
    SurfaceModel.cpp
    #Utils.cc
    EPBase.cpp
  )

  SET(SOURCE_H
    AddGroundTruth.h
    BoundaryDetector.h
    ContourDetector.h
    Relation.h
    SurfaceModel.hpp
    #Utils.hh
    EPBase.hpp
  )

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rPCLAddOns v4rEPUtils v4rTomGine v4ron_nurbs)
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ELSE (${PCL_FOUND})
  MESSAGE("\nERROR: SurfaceUtils: Could not locate PCL. Skip V4R_SURFACE_UTILS\n")
ENDIF(${PCL_FOUND})

ELSE(V4R_EPUTILS)
  MESSAGE(STATUS "********************************************\n   v4r/SurfaceUtils: missing V4R_EPUTILS\n   ********************************************\n")
ENDIF(V4R_EPUTILS)

ELSE(V4R_PCL_ADD_ONS)
  MESSAGE(STATUS "********************************************\n   v4r/SurfaceUtils: missing V4R_PCL_ADD_ONS\n   ********************************************\n")
ENDIF(V4R_PCL_ADD_ONS)

ENDIF(V4R_SURFACE_UTILS)

