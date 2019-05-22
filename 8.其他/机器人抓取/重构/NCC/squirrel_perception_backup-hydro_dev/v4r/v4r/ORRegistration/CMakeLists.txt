SET(V4R_ORREGISTRATION 1 CACHE BOOL "Registration module" )
IF(V4R_ORREGISTRATION)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

PROJECT(v4rORRegistration)

SET(SOURCE_CPP
  fast_icp_with_gc.cpp
  visibility_reasoning.cpp
)

SET(SOURCE_H
  fast_icp_with_gc.h
  fast_icp_with_gc.hpp
  uniform_sampling.h
  visibility_reasoning.h
  visibility_reasoning.hpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_ORREGISTRATION)
