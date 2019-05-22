cmake_minimum_required (VERSION 2.6)
SET(V4R_SURFACE_SEGMENTER 1 CACHE BOOL "Surface segmenter framework" )
IF(V4R_SURFACE_SEGMENTER)

    #IF(V4R_GRAPH_CUT)
    #IF(V4R_SVM)
    #IF(V4R_SURFACE_RELATIONS)
    #IF(V4R_SURFACE_MODELING)
    #IF(V4R_SURFACE_CLUSTERING)
    #IF(V4R_SURFACE_UTILS)
    #IF(V4R_EPUTILS)

include_directories(.)
include_directories(${OPENCV_INCLUDE_DIRS})

IF(${PCL_FOUND})

  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  PROJECT(v4rSurfaceSegmenter)
  SET(SOURCE_CPP
    segmentation.cpp
  ) 

  SET(SOURCE_H
    segmentation.hpp
  )

  add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
  target_link_libraries(${PROJECT_NAME} v4rSurfaceUtils v4rSurfaceClustering v4rEPUtils v4rSurfaceModeling v4rSurfaceRelations v4rsvm v4rGraphCut rt)
  v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ELSE (${PCL_FOUND})
MESSAGE("\nERROR: SurfaceSegmenter: Could not locate PCL. Skip V4R_SURFACE_SEGMENTER\n")
ENDIF(${PCL_FOUND})
  
#ELSE(V4R_EPUTILS)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceSegmenter: missing V4R_EPUTILS\n   ********************************************\n")
#ENDIF(V4R_EPUTILS)
#
#ELSE(V4R_SURFACE_UTILS)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceSegmenter: missing V4R_SURFACE_UTILS\n   ********************************************\n")
#ENDIF(V4R_SURFACE_UTILS)
#
#ELSE(V4R_SURFACE_CLUSTERING)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceSegmenter: missing V4R_SURFACE_CLUSTERING\n   ********************************************\n")
#ENDIF(V4R_SURFACE_CLUSTERING)
#
#ELSE(V4R_SURFACE_MODELING)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceSegmenter: missing V4R_SURFACE_MODELING\n   ********************************************\n")
#ENDIF(V4R_SURFACE_MODELING)
#
#ELSE(V4R_SURFACE_RELATIONS)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceSegmenter: missing V4R_SURFACE_RELATIONS\n   ********************************************\n")
#ENDIF(V4R_SURFACE_RELATIONS)
#
#ELSE(V4R_SVM)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceSegmenter: missing V4R_SVM\n   ********************************************\n")
#ENDIF(V4R_SVM)
#
#ELSE(V4R_GRAPH_CUT)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceSegmenter: missing V4R_GRAPH_CUT\n   ********************************************\n")
#ENDIF(V4R_GRAPH_CUT)

ENDIF(V4R_SURFACE_SEGMENTER)
