SET(V4R_PCORE 1 CACHE BOOL "Some simple methods (vector, matrix, line, plane)" )
IF(V4R_PCORE)

PROJECT(v4rPCore)

set(SOURCE_CPP 
 nrutil.c
 fitcircle.c
 Pose.cc
 PMath.cc
)

set(SOURCE_H 
 PHomography.hh
 PMath.hh
 PMatrix.ic
 PVector.hh
 nrutil.h
 PLine.hh
 PMath.ic
 PVector.ic
 fitcircle.h
 PCircle.hh
 PMatrix.hh
 Pose.hh
 toString.hpp
 PMath.hh
 PMath.ic
 PVector.hh
 PVector.ic
 PMatrix.hh
 PMatrix.ic
 PPlane.hh
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES})
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_PCORE)
