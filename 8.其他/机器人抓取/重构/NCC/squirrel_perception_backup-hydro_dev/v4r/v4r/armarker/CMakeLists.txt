SET(V4R_ARMARKER 1 CACHE BOOL "AR marker, pattern and pose detector" )
IF(V4R_ARMARKER)

PROJECT(ARMarker)

#FIND_PACKAGE (OpenCV QUIET)
#include_directories(${OPENCV_INCLUDE_DIRS})
#link_directories(${OPENCV_LIBRARY_DIRS})

include_directories("/usr/include/eigen3")

# Michael Zillich: this is ugly and thus removed
#IF ( "x${OPENCV_LIBRARIES}x" STREQUAL "xx"  )
#  SET ( OPENCV_LIBRARIES  cv cxcore highgui )
#  SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin")
#  SET(LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
#ENDIF("x${OPENCV_LIBRARIES}x" STREQUAL "xx" )

IF(UNIX AND NOT WIN32)
  FIND_PROGRAM(CMAKE_UNAME uname /bin /usr/bin /usr/local/bin )
  IF(CMAKE_UNAME)
    EXEC_PROGRAM(uname ARGS -m OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR)
    SET(CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE INTERNAL "processor type (i386 and x86_64)")
    IF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
      ADD_DEFINITIONS(-fPIC)
    ENDIF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  ENDIF(CMAKE_UNAME)
ENDIF(UNIX AND NOT WIN32)


set (ARMARKER_H  armarker.h ARComplexPatternDetector.hh ARPoseDetector.hh)
set (ARMARKER_CPP  detection.cpp pattern.cpp hdl.cpp ARComplexPatternDetector.cc ARPoseDetector.cc)

include_directories(./)
add_library(v4rmarker SHARED ${ARMARKER_H} ${ARMARKER_CPP})
target_link_libraries(v4rmarker ar ${OPENCV_LIBRARIES})

add_subdirectory(AR)
#add_subdirectory(examples)

INSTALL(DIRECTORY DESTINATION include/v4r/armarker)
INSTALL(TARGETS v4rmarker LIBRARY DESTINATION lib)
INSTALL(FILES ${ARMARKER_H} DESTINATION include/v4r/armarker)

ENDIF(V4R_ARMARKER)
