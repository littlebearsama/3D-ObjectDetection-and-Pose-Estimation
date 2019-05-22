SET(V4R_SVM 1 CACHE BOOL "Support Vector Machine - based on libsvm." )
SET(CMAKE_BUILD_TYPE "Debug")

IF(V4R_SVM)

PROJECT(v4rsvm)

IF(${PCL_FOUND})

  include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  SET(SOURCE_CPP
    svm.cpp
    SVMFileCreator.cpp
    SVMPredictorSingle.cpp
    SVMScale.cpp
    SVMTrainModel.cpp
    svmWrapper.cpp
  )

  SET(SOURCE_H
    svm.h
    SVMFileCreator.h
    SVMPredictorSingle.h
    SVMScale.h
    SVMTrainModel.h
    svmWrapper.h
  )

  add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
  target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
  v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ELSE (${PCL_FOUND})
MESSAGE("\nERROR: svm: Could not locate PCL. Skip V4R_SVM\n")
ENDIF(${PCL_FOUND})
  
ENDIF(V4R_SVM)

