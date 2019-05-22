SET(V4R_ATTENTION_MODULE 1 CACHE BOOL "Attention module." )
IF(V4R_ATTENTION_MODULE)

#FIND_PACKAGE (OpenCV QUIET)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

SET(CMAKE_BUILD_TYPE Release)

IF(V4R_EPUTILS)

PROJECT(v4rAttentionModule)

IF(${PCL_FOUND})

  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  add_definitions(-DUSE_PCL)

  SET(SOURCE_CPP_PCL
      SurfaceHeightMap.cpp
      RelativeSurfaceOrientationMap.cpp
      SurfaceCurvatureMap.cpp
      Symmetry3DMap.cpp
  )

  SET(SOURCE_H_PCL
      SurfaceHeightMap.hpp
      RelativeSurfaceOrientationMap.hpp
      SurfaceCurvatureMap.hpp
      Symmetry3DMap.hpp
  )

  SET(SOURCE_CPP
    WTA.cpp
    pyramidBase.cpp
    pyramidSimple.cpp
    pyramidItti.cpp
    pyramidFrintrop.cpp
    BaseMap.cpp
    ColorMap.cpp
    OrientationMap.cpp
    SymmetryMap.cpp
    IKNSaliencyMap.cpp
    FrintropSaliencyMap.cpp
    LocationMap.cpp
    MapsCombination.cpp
    TJ.cpp
    MSR.cpp
    HitRatio.cpp
  )
  SET(SOURCE_H
    WTA.hpp
    pyramidBase.hpp
    pyramidSimple.hpp
    pyramidItti.cpp
    pyramidFrintrop.cpp
    AttentionModuleErrors.hpp
    BaseMap.hpp
    ColorMap.hpp
    OrientationMap.hpp
    SymmetryMap.hpp
    IKNSaliencyMap.hpp
    FrintropSaliencyMap.hpp
    LocationMap.hpp
    MapsCombination.hpp
    AttentionModule.hpp
    headers.hpp
    TJ.hpp
    MSR.hpp
    HitRatio.hpp
  )

  add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP} ${SOURCE_CPP_PCL} ${SOURCE_H_PCL})
  target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} v4rEPUtils)
  v4r_add_library(${PROJECT_NAME} "${SOURCE_H};${SOURCE_H_PCL}")

  add_subdirectory(example)
  
ELSE (${PCL_FOUND})
  MESSAGE("\nERROR: AttentionModule: Could not locate PCL. Skip V4R_ATTENTION_MODULE\n")
ENDIF(${PCL_FOUND})

ELSE(V4R_EPUTILS)
  MESSAGE(STATUS "********************************************\n   v4r/AttentionModule warning: missing V4R_EPUTILS\n   ********************************************\n")
ENDIF(V4R_EPUTILS)

ENDIF(V4R_ATTENTION_MODULE)
