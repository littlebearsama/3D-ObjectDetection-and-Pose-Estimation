#find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ExmplOrientationMap ExmplOrientationMap.cpp)
target_link_libraries(ExmplOrientationMap ${OpenCV_LIBS} v4rEPUtils v4rAttentionModule)
 
add_executable(ExmplIttiSaliencyMap ExmplIttiSaliencyMap.cpp)
target_link_libraries(ExmplIttiSaliencyMap ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)
  
add_executable(ExmplFrintropSaliencyMap ExmplFrintropSaliencyMap.cpp)
target_link_libraries(ExmplFrintropSaliencyMap ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)

add_executable(Exmpl3DSymSalMap Exmpl3DSymSalMap.cpp AttentionExampleUtils.cpp AttentionExampleUtils.hpp)
target_link_libraries(Exmpl3DSymSalMap ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)
  
add_executable(ExmplSymSalMap ExmplSymSalMap.cpp)
target_link_libraries(ExmplSymSalMap ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)

add_executable(ExmplSalLine ExmplSalLine.cpp)
target_link_libraries(ExmplSalLine ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)

add_executable(ExmplWTA ExmplWTA.cpp)
target_link_libraries(ExmplWTA ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)

add_executable(ExmplMSR ExmplMSR.cpp)
target_link_libraries(ExmplMSR ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)

add_executable(ExmplColorMap ExmplColorMap.cpp)
target_link_libraries(ExmplColorMap ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)

add_executable(ExmplLocationMap ExmplLocationMap.cpp)
target_link_libraries(ExmplLocationMap ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)

add_executable(ExmplSurfHeightMap ExmplSurfHeightMap.cpp AttentionExampleUtils.cpp AttentionExampleUtils.hpp)
target_link_libraries(ExmplSurfHeightMap ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)

add_executable(ExmplSurfOrientationMap ExmplSurfOrientationMap.cpp AttentionExampleUtils.cpp AttentionExampleUtils.hpp)
target_link_libraries(ExmplSurfOrientationMap ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)
  
add_executable(ExmplSurfCurvatureMap ExmplSurfCurvatureMap.cpp)
target_link_libraries(ExmplSurfCurvatureMap ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)
 
add_executable(ExmplMapCombination ExmplMapCombination.cpp AttentionExampleUtils.cpp AttentionExampleUtils.hpp)
target_link_libraries(ExmplMapCombination ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)
  
add_executable(ExmplHitRatio ExmplHitRatio.cpp)
target_link_libraries(ExmplHitRatio ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)
  
add_executable(DrawStrokesForGulshan DrawStrokesForGulshan.cpp)
target_link_libraries(DrawStrokesForGulshan ${OpenCV_LIBS} ${PCL_LIBRARIES} v4rEPUtils v4rAttentionModule)

