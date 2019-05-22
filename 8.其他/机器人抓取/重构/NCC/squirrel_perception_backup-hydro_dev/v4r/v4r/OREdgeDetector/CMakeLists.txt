SET(V4R_OREDGEDETECTOR 1 CACHE BOOL "Detect various types of edges on organized point clouds" )
IF(V4R_OREDGEDETECTOR)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

PROJECT(v4rOREdgeDetector)

SET(SOURCE_CPP
  convolution_2d.cpp
  organized_edge_detection.cpp
)

SET(SOURCE_H
  kernel.h
  kernel.hpp
  edge.h
  edge.hpp
  convolution.h
  convolution.hpp
  organized_edge_detection.h
  organized_edge_detection.hpp
)


add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})

v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_OREDGEDETECTOR)



