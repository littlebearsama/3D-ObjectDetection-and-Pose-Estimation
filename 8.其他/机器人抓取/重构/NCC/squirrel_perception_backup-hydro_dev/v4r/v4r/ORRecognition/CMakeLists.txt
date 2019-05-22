SET(V4R_ORRECOGNITION 1 CACHE BOOL "Recognition module - HV and correspondence grouping" )
IF(V4R_ORRECOGNITION)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

PROJECT(v4rORRecognition)

SET(SOURCE_CPP_HV
  ghv_opt.cpp
  ghv.cpp
  hv_go_3D.cpp
#  hv_go_1.cpp
)

SET(SOURCE_H_HV
  ghv.h
  ghv.hpp
  ghv_opt.h
  ghv_opt.hpp
  hv_go_3D.h
#  hv_go_1.h
#  hv_go_1.hpp
  hypotheses_verification.h
  occlusion_reasoning.h
  occlusion_reasoning.hpp
)

SET(SOURCE_H_CG
  correspondence_grouping.h
  correspondence_grouping.hpp
  geometric_consistency.h
  geometric_consistency.hpp
  hough_3d.h
  hough_3d.hpp
  graph_geometric_consistency.h
  graph_geometric_consistency.hpp


  kmeans.h
  kmeans.hpp
)

SET(SOURCE_CPP_CG
  geometric_consistency.cpp
  hough_3d.cpp
  graph_geometric_consistency.cpp

  kmeans.cpp
)


add_library(${PROJECT_NAME} SHARED ${SOURCE_H_HV} ${SOURCE_CPP_HV} ${SOURCE_H_CG} ${SOURCE_CPP_CG})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
v4r_add_library(${PROJECT_NAME} "${SOURCE_H_HV} ${SOURCE_H_CG}")

ENDIF(V4R_ORRECOGNITION)



