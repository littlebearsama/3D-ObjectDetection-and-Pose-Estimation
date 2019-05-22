SET(V4R_EPUTILS 1 CACHE BOOL "Different utils functions for reading/writing/conversions." )
IF(V4R_EPUTILS)

#FIND_PACKAGE (OpenCV QUIET)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

PROJECT(v4rEPUtils)

IF(${PCL_FOUND})
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  SET(SOURCE_CPP
    utils.cpp
    algo.cpp
    convertions.cpp
    normalization.cpp
    #sphereHistogram.cpp
    PCA.cpp
    math.cpp
    connectedComponents.cpp
    drawUtils.cpp
    #debugUtils.cpp
    timeUtils.cpp
   )
  SET(SOURCE_H
    utils.hpp
    algo.hpp
    convertions.hpp
    normalization.hpp
    headers.hpp
    EPUtils.hpp
    PCLPreprocessingXYZRC.hpp
    #sphereHistogram.hpp
    PCA.hpp
    math.hpp
    connectedComponents.hpp
    drawUtils.hpp
    #debugUtils.hpp
    timeUtils.hpp
   )

  add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES}  ${PCL_LIBRARIES})
  v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

  add_subdirectory(example)
  
ELSE (${PCL_FOUND})
  MESSAGE("\nERROR: EPUtils: Could not locate PCL. Skip V4R_EPUTILS\n")
ENDIF(${PCL_FOUND})

ENDIF(V4R_EPUTILS)
