SET(V4R_KEYPOINT_CAMERA_TRACKER 1 CACHE BOOL "Keypoint based camera tracker (monocular and stereo/ RGBD version)" )
IF(V4R_KEYPOINT_CAMERA_TRACKER)

#FIND_PACKAGE (OpenCV)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})


FIND_PACKAGE(Ceres)
IF(${Ceres_FOUND})
  INCLUDE_DIRECTORIES(${CERES_INCLUDES})
ELSE(${Ceres_FOUND})
  MESSAGE("\nATTENTION: Ceres is not installed! StructureFromMotionInitAR will not work properly!\n")
  add_definitions(-DKP_NO_CERES_AVAILABLE)
ENDIF(${Ceres_FOUND})

find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

IF(NOT V4R_ARMARKER)
  MESSAGE("\nATTENTION: ARMarker is not aktivated! StructureFromMotionInitAR will not work!\n")
  add_definitions(-DKP_NO_ARMARKER)
ENDIF(NOT V4R_ARMARKER)

PROJECT(v4rKeypointCameraTracker)
SET(SOURCE_CPP
  CameraTrackerPnP.cc
  CameraTrackerRGBD.cc
  KeypointTracker.cc
  LoopClosingRT.cc
  Scene.cc
  StructureFromMotionInitAR.cc
  Triangulation.cc
  View.cc
)
SET(SOURCE_H
  ARMarkerDetector.hh
  BundleAdjusterRT.hh
  CameraTrackerPnP.hh
  CameraTrackerRGBD.hh
  KeypointTracker.hh
  LoopClosingRT.hh
  ProjBundleAdjuster.hh
  Scene.cc
  StructureFromMotionInitAR.hh
  Triangulation.hh
  View.hh
)
IF(V4R_ARMARKER)
  SET(SOURCE_CPP ${SOURCE_CPP}
    ARMarkerDetector.cc
  )
ENDIF(V4R_ARMARKER)
IF(${Ceres_FOUND})
  SET(SOURCE_CPP ${SOURCE_CPP} 
    BundleAdjusterRT.cc
    ProjBundleAdjuster.cc
  )
ENDIF(${Ceres_FOUND})



add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})

target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} v4rKeypointTools v4rKeypointBase)

# in case of ceres is a static lib it must be linked to the executable
IF(${Ceres_FOUND})
get_target_property(LIBTYPE ${CERES_LIBRARIES} TYPE)
IF("${LIBTYPE}" STREQUAL "SHARED_LIBRARY")
  target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
ENDIF("${LIBTYPE}" STREQUAL "SHARED_LIBRARY")
ENDIF(${Ceres_FOUND})

IF(V4R_ARMARKER)
  target_link_libraries(${PROJECT_NAME} v4rmarker)
ENDIF(V4R_ARMARKER)

v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")


ENDIF(V4R_KEYPOINT_CAMERA_TRACKER)



