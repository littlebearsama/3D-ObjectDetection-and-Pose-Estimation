SET(V4R_SURFACE_MODELING 1 CACHE BOOL "Model selection and parametric surface estimation (planes and NURBS)." )
IF(V4R_SURFACE_MODELING)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

#IF(V4R_PCL_ADD_ONS)
#IF(V4R_EPUTILS)
#IF(V4R_SURFACE_UTILS)

PROJECT(v4rSurfaceModeling)

IF(${PCL_FOUND})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  SET(SOURCE_CPP
    SurfaceModeling.cc
  )

  SET(SOURCE_H
    SurfaceModeling.hh
  )

  add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
  target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rSurfaceUtils v4rPCLAddOns v4rEPUtils)
  v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ELSE (${PCL_FOUND})
  MESSAGE("\nERROR: SurfaceModeling: Could not locate PCL. Skip V4R_SURFACE_MODELING\n")
ENDIF(${PCL_FOUND})
  
#ELSE(V4R_SURFACE_UTILS)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceModeling: missing V4R_SURFACE_UTILS\n   ********************************************\n")
#ENDIF(V4R_SURFACE_UTILS)
#
#ELSE(V4R_EPUTILS)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceModeling: missing V4R_EPUTILS\n   ********************************************\n")
#ENDIF(V4R_EPUTILS)
#
#ELSE(V4R_PCL_ADD_ONS)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceModeling: missing V4R_PCL_ADD_ONS\n   ********************************************\n")
#ENDIF(V4R_PCL_ADD_ONS)

ENDIF(V4R_SURFACE_MODELING)

