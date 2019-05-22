SET(V4R_KEYPOINT_CAMERA_TRACKER_PCL 1 CACHE BOOL "Keypoint based camera tracker (PCL wrapper)" )
IF(V4R_KEYPOINT_CAMERA_TRACKER_PCL)
IF(V4R_KEYPOINT_CAMERA_TRACKER)

FIND_PACKAGE(Ceres)
IF(${Ceres_FOUND})
  INCLUDE_DIRECTORIES(${CERES_INCLUDES})
ELSE(${Ceres_FOUND})
  MESSAGE("\nATTENTION: Ceres is not installed! StructureFromMotionInitAR will not work properly!\n")
  add_definitions(-DKP_NO_CERES_AVAILABLE)
ENDIF(${Ceres_FOUND})

#FIND_PACKAGE (OpenCV)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


PROJECT(v4rKeypointCameraTrackerPCL)
SET(SOURCE_CPP
  CameraTrackerRGBDPCL.cc
)

SET(SOURCE_H
  CameraTrackerRGBDPCL.hh
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} v4rKeypointCameraTracker v4rKeypointTools)
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_KEYPOINT_CAMERA_TRACKER)
ENDIF(V4R_KEYPOINT_CAMERA_TRACKER_PCL)

