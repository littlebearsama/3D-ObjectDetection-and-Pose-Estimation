SET(V4R_ORGT_EVALUATOR 1 CACHE BOOL "Ground-truth evaluator tools for object recognition" )
IF(V4R_ORGT_EVALUATOR)

include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

PROJECT(v4rORGTEvaluator)

SET(SOURCE_CPP
  or_evaluator.cpp
)

SET(SOURCE_H
  or_evaluator.h
  or_evaluator.hpp
)


add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rORUtils)

v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_ORGT_EVALUATOR)



