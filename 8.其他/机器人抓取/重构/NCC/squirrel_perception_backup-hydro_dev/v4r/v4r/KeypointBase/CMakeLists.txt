SET(V4R_KEYPOINT_BASE 1 CACHE BOOL "Keypoint related data types, detectors, descriptors" )
IF(V4R_KEYPOINT_BASE)

#FIND_PACKAGE (OpenCV)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})
#include_directories("/usr/include/eigen3")

find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# define shader path
set(KEYPOINT_BASE_SHADER ${CMAKE_INSTALL_PREFIX}/include/v4r/KeypointBase/shader/ CACHE PATH "Path to shader files")
set(KEYPOINT_BASE_SHADER ${KEYPOINT_BASE_SHADER}/)
add_definitions(-DKEYPOINT_BASE_SHADER="${KEYPOINT_BASE_SHADER}")
message( STATUS "KEYPOINT_BASE_SHADER is ${KEYPOINT_BASE_SHADER}")

find_file(OCV_NONFREE_HEADER opencv2/nonfree/features2d.hpp)
IF(OCV_NONFREE_HEADER)
  SET(HAVE_OPENCV_NONFREE 1)
ENDIF()

PROJECT(v4rKeypointBase)
SET(SOURCE_CPP
  ComputeImGDescOrientations.cc
  ComputeImGradientDescriptors.cc
  Feature.cc
  FeatureDetector_K_HARRIS.cc
  FeatureDetector_KD_SIFTGPU.cc
  FeatureDetector_D_FREAK.cc
  FeatureDetector_KD_ORB.cc
  FeatureDetector_KD_FAST_IMGD.cc
  ImGradientDescriptor.cc
  ImGDescOrientation.cc
  LinkedKeypoint.cc
  OptimizePointsMultiViewGD.cc
  PSiftGPU.cc
  RefinePatchLocationLK.cc
  RefineProjectedPointLocationLK.cc
)
IF (HAVE_OPENCV_NONFREE)
SET(SOURCE_CPP ${SOURCE_CPP}
  FeatureDetector_KD_CVSURF.cc
  FeatureDetector_KD_CVSIFT.cc
)
ENDIF(HAVE_OPENCV_NONFREE)
IF (V4R_TOMGINE_5)
SET(SOURCE_CPP ${SOURCE_CPP} RefineProjectedPointLocationLKGPU.cc)
ENDIF(V4R_TOMGINE_5)

SET(SOURCE_H
  ComputeImGDescOrientations.hh
  ComputeImGradientDescriptors.hh
  Feature.hh
  FeatureMatch.hh
  FeatureDetector.hh
  FeatureDetector_K_HARRIS.hh
  FeatureDetector_KD_SIFTGPU.hh
  FeatureDetector_D_FREAK.hh
  FeatureDetector_KD_ORB.hh
  FeatureDetector_KD_FAST_IMGD.hh
  ImGradientDescriptor.hh
  ImGDescOrientation.hh
  LinkedKeypoint.hh
  MultiViewPatchError.hpp
  OptimizePointsMultiViewGD.hh
  PSiftGPU.hh
  RefinePatchLocationLK.hh
  RefineProjectedPointLocationLK.hh
  RefineProjectedPointLocationLKbase.hh
)
IF (HAVE_OPENCV_NONFREE)
SET(SOURCE_H ${SOURCE_H}
  FeatureDetector_KD_CVSURF.hh
  FeatureDetector_KD_CVSIFT.hh
)
ENDIF(HAVE_OPENCV_NONFREE)

IF (V4R_TOMGINE_5)
SET(SOURCE_H ${SOURCE_H} RefineProjectedPointLocationLKGPU.hh)
ENDIF(V4R_TOMGINE_5)

set(SOURCE_SHADER
  shader/hread.fsh
  shader/hread.vsh
  shader/tgtread.fsh
  shader/tgtread.vsh
  shader/gxyerr.fsh
  shader/gxyerr.vsh
  shader/ncc.fsh
  shader/ncc.vsh
  shader/solveTF.vsh
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP} ${SOURCE_SHADER})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} siftgpu v4rKeypointTools)
IF (V4R_TOMGINE_5)
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} v4rTomGine5)
ENDIF(V4R_TOMGINE_5)

v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_KEYPOINT_BASE)

