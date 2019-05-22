cmake_minimum_required ( VERSION 2.6 )
PROJECT(v4r)
find_package(catkin REQUIRED)
#find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_CXX_FLAGS "-ggdb -O3 -fPIC -march=native -mno-avx")
###########################################
## Do not waste the top level cmake !!! ###
###########################################
# the gcc-4.2.1 coming with MacOS X is not compatible with the OpenMP pragmas we use, so disabling OpenMP for it
if((NOT APPLE) OR (NOT CMAKE_COMPILER_IS_GNUCXX) OR (GCC_VERSION VERSION_GREATER 4.2.1))
    find_package(OpenMP)
endif()
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message (STATUS "Found OpenMP")
    if(MSVC90 OR MSVC10)
        if(MSVC90)
            set(OPENMP_DLL VCOMP90)
        elseif(MSVC10)
            set(OPENMP_DLL VCOMP100)
        endif(MSVC90)
        set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} /DELAYLOAD:${OPENMP_DLL}D.dll")
        set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DELAYLOAD:${OPENMP_DLL}.dll")
    endif(MSVC90 OR MSVC10)
else(OPENMP_FOUND)
    message (STATUS "Not found OpenMP")
endif()

include(openmp.cmake)
include(opencv.cmake)
include(pcl.cmake)
include(v4rutils.cmake)

SET(LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin")

set(V4R_BUILD 1)

#add_definitions(-DDEBUG) # this should be defined in each module individually [tm]
add_definitions(-DLINUX)
add_definitions("-Wall -Wno-unused-function -Wl,--no-undefined -fno-strict-aliasing -pthread")

include_directories(.)

#add_subdirectory(apps)
#add_subdirectory(ak)
add_subdirectory(v4rexternal)
add_subdirectory(v4r)

message(STATUS "V4R_LIBRARIES: " ${V4R_LIBRARIES})
message(STATUS "V4R_LIBRARIES_PC: " ${V4R_LIBRARIES_PC})

SET(V4R_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE INTERNAL "includes dir" FORCE)

catkin_package(
    INCLUDE_DIRS ${V4R_INCLUDE_DIR}
    LIBRARIES ${V4R_LIBS}
)
LIST(APPEND STRANDSV4R_LIBSS "v4rAttentionModule")
LIST(APPEND STRANDSV4R_LIBSS "v4rGraphCut")
LIST(APPEND STRANDSV4R_LIBSS "v4rPCore")
LIST(APPEND STRANDSV4R_LIBSS "v4rSurfaceModeling")
LIST(APPEND STRANDSV4R_LIBSS "v4rSurfaceUtils")
LIST(APPEND STRANDSV4R_LIBSS "v4rTomGine")
#LIST(APPEND STRANDSV4R_LIBSS "v4rTomGine5")
LIST(APPEND STRANDSV4R_LIBSS "v4rTomGinePCL")
#LIST(APPEND STRANDSV4R_LIBSS "v4rCalibrationRGBD")
LIST(APPEND STRANDSV4R_LIBSS "v4rEPUtils")
LIST(APPEND STRANDSV4R_LIBSS "v4rSurfaceClustering")
LIST(APPEND STRANDSV4R_LIBSS "v4rSurfaceRelations")
LIST(APPEND STRANDSV4R_LIBSS "v4ron_nurbs")
LIST(APPEND STRANDSV4R_LIBSS "v4rSVM")
LIST(APPEND STRANDSV4R_LIBSS "v4rsvm")
LIST(APPEND STRANDSV4R_LIBSS "v4rKeypointBase")
LIST(APPEND STRANDSV4R_LIBSS "v4rKeypointCameraTracker")
LIST(APPEND STRANDSV4R_LIBSS "v4rKeypointCameraTrackerPCL")
LIST(APPEND STRANDSV4R_LIBSS "v4rKeypointTools")
#LIST(APPEND STRANDSV4R_LIBSS "v4rKeypointConversions")
LIST(APPEND STRANDSV4R_LIBSS "v4rEDT")
LIST(APPEND STRANDSV4R_LIBSS "v4rORUtils")
LIST(APPEND STRANDSV4R_LIBSS "v4rORFramework")
LIST(APPEND STRANDSV4R_LIBSS "v4rORRecognition")
LIST(APPEND STRANDSV4R_LIBSS "v4rORRegistration")
LIST(APPEND STRANDSV4R_LIBSS "v4rSurfaceSegmenter")

set(STRANDSV4R_LIBS ${STRANDSV4R_LIBSS} CACHE STRING "STRANDSV4R LIBS" FORCE)

# install target
configure_file(v4r.pc.in "${PROJECT_BINARY_DIR}/v4r.pc" @ONLY)
INSTALL(FILES ${PROJECT_BINARY_DIR}/v4r.pc DESTINATION lib/pkgconfig)
# uninstall target
configure_file(
"${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
"${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
IMMEDIATE @ONLY)
add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
