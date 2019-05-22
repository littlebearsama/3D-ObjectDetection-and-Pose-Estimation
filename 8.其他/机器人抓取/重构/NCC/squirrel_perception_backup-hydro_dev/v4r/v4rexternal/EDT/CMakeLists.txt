SET(EXT_EDT 1 CACHE BOOL "Distance transform module" )
IF(EXT_EDT)

PROJECT(v4rEDT)
SET(SOURCE_CPP
    propagation_distance_field.cpp
)

SET(SOURCE_H
  distance_field.h
  voxel_grid.h
  propagation_distance_field.h
)

include_directories(${PCL_INCLUDE_DIRS})
add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(EXT_EDT)

