#find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ExmplDrawPoints ExmplDrawPoints.cpp)
target_link_libraries(ExmplDrawPoints ${OpenCV_LIBS} v4rEPUtils)
  
add_executable(ExmplXYZRGB2Disparity ExmplXYZRGB2Disparity.cpp)
target_link_libraries(ExmplXYZRGB2Disparity ${OpenCV_LIBS} v4rEPUtils)
