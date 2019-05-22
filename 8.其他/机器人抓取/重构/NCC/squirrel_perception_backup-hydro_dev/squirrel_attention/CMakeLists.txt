cmake_minimum_required(VERSION 2.8.3)
project(squirrel_attention)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs geometry_msgs cv_bridge squirrel_object_perception_msgs v4r) #pcl_conversions

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES squirrel_attention
 CATKIN_DEPENDS squirrel_object_perception_msgs v4r
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#message(STATUS "MY PCL_INCLUDE_DIRS" ${PCL_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${V4R_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

## Declare a cpp library
# add_library(squirrel_attention
#   src/${PROJECT_NAME}/squirrel_attention.cpp
# )

#message(STATUS "MY V4R_LIBS" ${V4R_LIBS})
# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#     message(STATUS "MY ${_variableName}=${${_variableName}}")
# endforeach()
# link_directories(${V4R_LIB_DIR})

## Declare a cpp executable
# add_executable(squirrel_attention_node src/squirrel_attention_node.cpp)
add_executable(squirrel_attention_location src/squirrel_attention_location.cpp)
add_executable(squirrel_attention_color src/squirrel_attention_color.cpp)
add_executable(squirrel_attention_itti src/squirrel_attention_itti.cpp)
add_executable(squirrel_attention_symmetry src/squirrel_attention_symmetry.cpp)
add_executable(squirrel_attention_3Dsymmetry src/squirrel_attention_3Dsymmetry.cpp)
add_executable(squirrel_attention_height src/squirrel_attention_height.cpp)
add_executable(squirrel_attention_surfaceorientation src/squirrel_attention_surfaceorientation.cpp)
#add_executable(squirrel_attention_test src/test.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(squirrel_attention_node squirrel_attention_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(squirrel_attention_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(squirrel_attention_location ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
add_dependencies(squirrel_attention_location v4r)

target_link_libraries(squirrel_attention_color ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
add_dependencies(squirrel_attention_color v4r)

target_link_libraries(squirrel_attention_itti ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
add_dependencies(squirrel_attention_itti v4r)

target_link_libraries(squirrel_attention_symmetry ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
add_dependencies(squirrel_attention_symmetry v4r)

target_link_libraries(squirrel_attention_3Dsymmetry ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
add_dependencies(squirrel_attention_3Dsymmetry v4r)

target_link_libraries(squirrel_attention_height ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
add_dependencies(squirrel_attention_height v4r)

target_link_libraries(squirrel_attention_surfaceorientation ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
add_dependencies(squirrel_attention_surfaceorientation v4r)

#target_link_libraries(squirrel_attention_test ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${STRANDSV4R_LIBS})
#add_dependencies(squirrel_attention_test v4r)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS squirrel_attention squirrel_attention_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############
find_package(rostest REQUIRED)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(squirrel_attention_location_test test/squirrel_attention_location_test.cpp)
  if(TARGET squirrel_attention_location_test)
    target_link_libraries(squirrel_attention_location_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  add_rostest(test/squirrel_attention_location_test.test)

  catkin_add_gtest(squirrel_attention_color_test test/squirrel_attention_color_test.cpp)
  if(TARGET squirrel_attention_color_test)
    target_link_libraries(squirrel_attention_color_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  add_rostest(test/squirrel_attention_color_test.test)

  catkin_add_gtest(squirrel_attention_itti_test test/squirrel_attention_itti_test.cpp)
  if(TARGET squirrel_attention_itti_test)
    target_link_libraries(squirrel_attention_itti_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  add_rostest(test/squirrel_attention_itti_test.test)
  
  catkin_add_gtest(squirrel_attention_symmetry_test test/squirrel_attention_symmetry_test.cpp)
  if(TARGET squirrel_attention_symmetry_test)
    target_link_libraries(squirrel_attention_symmetry_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  add_rostest(test/squirrel_attention_symmetry_test.test)
  
  catkin_add_gtest(squirrel_attention_3Dsymmetry_test test/squirrel_attention_3Dsymmetry_test.cpp)
  if(TARGET squirrel_attention_3Dsymmetry_test)
    target_link_libraries(squirrel_attention_3Dsymmetry_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  add_rostest(test/squirrel_attention_3Dsymmetry_test.test)
  
  catkin_add_gtest(squirrel_attention_height_test test/squirrel_attention_height_test.cpp)
  if(TARGET squirrel_attention_height_test)
    target_link_libraries(squirrel_attention_height_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  add_rostest(test/squirrel_attention_height_test.test)
  
  catkin_add_gtest(squirrel_attention_surfaceorientation_test test/squirrel_attention_surfaceorientation_test.cpp)
  if(TARGET squirrel_attention_surfaceorientation_test)
    target_link_libraries(squirrel_attention_surfaceorientation_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${STRANDSV4R_LIBS})
  endif()
  add_rostest(test/squirrel_attention_surfaceorientation_test.test)
endif()
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_squirrel_attention.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
