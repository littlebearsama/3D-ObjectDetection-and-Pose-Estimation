SET(V4R_SURFACE_CLUSTERING 1 CACHE BOOL "Normals-clustering of point cloud." )
IF(V4R_SURFACE_CLUSTERING)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

#IF(V4R_EPUTILS)
#IF(V4R_SURFACE_UTILS)
#IF(V4R_ON_NURBS)

PROJECT(v4rSurfaceClustering)

IF(${PCL_FOUND})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  SET(SOURCE_CPP
    ClusterNormalsToPlanes.cc
    ZAdaptiveNormals.cc
  )

  SET(SOURCE_H
    ClusterNormalsToPlanes.hh
    PPlane.h
    ZAdaptiveNormals.hh
  )

  add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
  target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rEPUtils v4rSurfaceUtils v4ron_nurbs)
  v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ELSE (${PCL_FOUND})
  MESSAGE("\nERROR: SurfaceClustering: Could not locate PCL. Skip V4R_SURFACE_CLUSTERING\n")
ENDIF(${PCL_FOUND})
  
#ELSE(V4R_ON_NURBS)
#  MESSAGE(STATUS "********************************************\n v4r/SurfaceClustering: missing V4R_ON_NURBS\n   ********************************************\n")
#ENDIF(V4R_ON_NURBS)
#
#ELSE(V4R_SURFACE_UTILS)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceClustering: missing V4R_SURFACE_UTILS\n   ********************************************\n")
#ENDIF(V4R_SURFACE_UTILS)
#
#ELSE(V4R_EPUTILS)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceClustering: missing V4R_EPUTILS\n   ********************************************\n")
#ENDIF(V4R_EPUTILS)

ENDIF(V4R_SURFACE_CLUSTERING)

