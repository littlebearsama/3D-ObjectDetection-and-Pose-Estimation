SET(V4R_GRAPH_CUT 1 CACHE BOOL "Graph cut alogithm - based on Felzenszwalb implementation." )
IF(V4R_GRAPH_CUT)

PROJECT(v4rGraphCut)

IF(${PCL_FOUND})

  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  include_directories(.)

  # Set the sources.
  SET(SOURCE_H
    disjoint-set.h
    Edge.h
    GraphCut.h
    Graph.h
  )

  SET(SOURCE_CPP
    disjoint-set.cpp
    GraphCut.cpp
    Graph.cpp
  )

  add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
  target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
  v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ELSE (${PCL_FOUND})
MESSAGE("\nERROR: GraphCut: Could not locate PCL. Skip V4R_GRAPH_CUT\n")
ENDIF(${PCL_FOUND})

ENDIF(V4R_GRAPH_CUT)
