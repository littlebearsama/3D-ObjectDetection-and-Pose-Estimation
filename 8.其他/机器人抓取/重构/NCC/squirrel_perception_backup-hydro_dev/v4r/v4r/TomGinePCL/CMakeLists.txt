cmake_minimum_required(VERSION 2.8)

set(V4R_TOMGINE_PCL 1 CACHE BOOL "TomGine Wrapper for PCL types (requires Point-Cloud Library)" )
IF(V4R_TOMGINE_PCL)
project(v4rTomGinePCL)

# create tomgine include directory name
string(REGEX REPLACE "v4r" "" TOMGINE_INCLUDE_NAME ${PROJECT_NAME})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(SOURCE_CPP 
	tgPCL2TomGine.cpp
	tgTomGineThreadPCL.cpp
	)

set(SOURCE_H
	tgPCL2TomGine.h
	tgTomGineThreadPCL.h
	)


add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} v4rTomGine ${PCL_LIBRARIES})
	
if(${V4R_BUILD})

	message(STATUS "TomGineIP: Building within v4r.")
	v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

else(${V4R_BUILD})

  install(DIRECTORY DESTINATION include/v4r/${TOMGINE_INCLUDE_NAME})
  install(FILES ${SOURCE_H} DESTINATION include/v4r/${TOMGINE_INCLUDE_NAME})
  install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

endif(${V4R_BUILD})


ENDIF(V4R_TOMGINE_PCL)
