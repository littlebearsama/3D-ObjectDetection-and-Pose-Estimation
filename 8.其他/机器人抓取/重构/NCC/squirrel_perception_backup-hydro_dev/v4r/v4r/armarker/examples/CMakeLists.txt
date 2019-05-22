include_directories(${V4R_INCLUDE})

ADD_EXECUTABLE(armarker_sample main.cpp)
target_link_libraries(armarker_sample ${OPENCV_LIBRARIES} v4rmarker)


ADD_EXECUTABLE(armarker_pose main_marker_pose.cpp)
target_link_libraries(armarker_pose ${OPENCV_LIBRARIES} v4rmarker)

# ADD_EXECUTABLE(armarker_fw main_marker_fw.cpp)
# target_link_libraries(armarker_fw v4rmarker v4rfw ${OPENCV_LIBRARIES})
 
