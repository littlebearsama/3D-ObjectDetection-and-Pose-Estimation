SET(V4R_ORFRAMEWORK 1 CACHE BOOL "Recognition framework" )
IF(V4R_ORFRAMEWORK)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

PROJECT(v4rORFramework)

SET(SOURCE_H_UTILS
  persistence_utils.h
  metrics.h
  vtk_model_sampling.h
  faat_3d_rec_framework_defines.h
  voxel_based_correspondence_estimation.h
  multiplane_segmentation.h
  voxel_based_correspondence_estimation.hpp
  multiplane_segmentation.hpp
)

SET(SOURCE_CPP_UTILS
  voxel_based_correspondence_estimation.cpp
  multiplane_segmentation.cpp
)

SET(SOURCE_H_DATA_SOURCES
  source.h
  unregistered_views_source.h
  partial_pcd_source.h  
  partial_pcd_source.hpp
  model_only_source.h
  registered_views_source.h
  registered_views_pp_source.h
  source2d.h
  mesh_source.h
)

SET(SOURCE_CPP_DATA_SOURCES
  mesh_source.cpp             
  model_only_source.cpp                           
  partial_pcd_source.cpp               
)

SET(SOURCE_H_PIPELINES
  recognizer.h
  global_nn_classifier.h           
  local_recognizer.h
  multi_pipeline_recognizer.h                 
  #linemod3d_recognizer.h
  hough_grouping_local_recognizer.h
  #implementations
  global_nn_classifier.hpp
  global_nn_recognizer_crh.hpp   
  multi_pipeline_recognizer.hpp
  local_recognizer.hpp
  global_nn_recognizer_cvfh.hpp
)

SET(SOURCE_CPP_PIPELINES
 multi_pipeline_recognizer.cpp               
 #linemod3d_recognizer.cpp #only works with PCL-trunk (not in releases)
 global_nn_classifier.cpp         
 local_recognizer.cpp
 global_nn_recognizer_cvfh.cpp                          
)

SET(SOURCE_H_FEATURES
  #3D LOCAL
  local_estimator.h 
  normal_estimator.h
  shot_local_estimator.h
  shot_local_estimator_omp.h
  colorshot_local_estimator.h
  #3D GLOBAL
  global_estimator.h
  global_nn_recognizer_cvfh.h
  organized_ourcvfh.h
  color_ourcvfh_estimator.h  
  ourcvfh_estimator.h
  crh_estimator.h  
  cvfh_estimator.h
  esf_estimator.h
  vfh_estimator.h
  organized_color_ourcvfh_estimator.h  
  #image based
  image_local_estimator.h
  vedaldi_sift_local_estimator.h
  sift_local_estimator.h
  opencv_sift_local_estimator.h
)

SET(SOURCE_CPP_FEATURES
  local_estimator.cpp
  shot_local_estimator.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H_UTILS} ${SOURCE_CPP_UTILS} ${SOURCE_H_FEATURES} ${SOURCE_CPP_FEATURES}
                                   ${SOURCE_H_DATA_SOURCES} ${SOURCE_CPP_DATA_SOURCES} ${SOURCE_H_PIPELINES} ${SOURCE_CPP_PIPELINES}
                                   )
                                   
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} v4rEDT)
v4r_add_library(${PROJECT_NAME} "${SOURCE_H_UTILS} ${SOURCE_CPP_UTILS} ${SOURCE_H_FEATURES} ${SOURCE_CPP_FEATURES} ${SOURCE_H_DATA_SOURCES} ${SOURCE_CPP_DATA_SOURCES} ${SOURCE_H_PIPELINES} ${SOURCE_CPP_PIPELINES}")

ENDIF(V4R_ORFRAMEWORK)



