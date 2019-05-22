SET(V4R_SURFACE_RELATIONS 1 CACHE BOOL "Relation estimation between parametric surface patches." )
IF(V4R_SURFACE_RELATIONS)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")


#IF(V4R_EPUTILS)
#IF(V4R_SURFACE_UTILS)

PROJECT(v4rSurfaceRelations)

IF(${PCL_FOUND})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  SET(SOURCE_CPP
    #AssemblyRelations.cpp
    BoundaryRelationsBase.cpp
    BoundaryRelationsMeanDepth.cpp
    BoundaryRelationsMeanCurvature.cpp
    BoundaryRelationsMeanColor.cpp
    ColorHistogram.cpp
    #ContourNormalsDistance.cc
    cvgabor.cpp
    Fourier.cpp
    Gabor.cpp
    StructuralRelations.cpp
    Texture.cpp
    #Vs3ArcRelations.cpp
  )

  SET(SOURCE_H
    #AssemblyRelations.h
    BoundaryRelationsBase.hpp
    BoundaryRelationsMeanDepth.hpp
    BoundaryRelationsMeanCurvature.hpp
    BoundaryRelationsMeanColor.hpp
    ColorHistogram.h
    #ContourNormalsDistance.hh
    cvgabor.h
    Fourier.h
    Gabor.h
    StructuralRelations.h
    Texture.h
    #Vs3ArcRelations.h
    Relation.h
  )

  add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
  target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rSurfaceUtils v4rEPUtils)
  v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ELSE (${PCL_FOUND})
  MESSAGE("\nERROR: SurfaceRelations: Could not locate PCL. Skip V4R_SURFACE_RELATIONS\n")
ENDIF(${PCL_FOUND})
  
#ELSE(V4R_SURFACE_UTILS)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceRelations: missing V4R_SURFACE_UTILS\n   ********************************************\n")
#ENDIF(V4R_SURFACE_UTILS)
#
#ELSE(V4R_EPUTILS)
#  MESSAGE(STATUS "********************************************\n   v4r/SurfaceRelations: missing V4R_EPUTILS\n   ********************************************\n")
#ENDIF(V4R_EPUTILS)

ENDIF(V4R_SURFACE_RELATIONS)

