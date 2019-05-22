cmake_minimum_required(VERSION 2.8)
set(V4R_TOMGINE_5 0 CACHE BOOL "OpenGL-GPU rendering, geometry and image processing library." )

IF(V4R_TOMGINE_5)

project(v4rTomGine5)

# pkg config
find_package(PkgConfig REQUIRED)
pkg_search_module(GLU REQUIRED glu)


# find OpenGL
pkg_search_module(GL REQUIRED gl)
if(NOT GL_FOUND)
        message(FATAL "libgl not found, required by v4rTomGine5")
endif(NOT GL_FOUND)

# find GLEW
pkg_search_module(GLEW 1.10 REQUIRED glew)
if(NOT GL_FOUND)
        message(FATAL "libGLEW not found, required by v4rTomGine5")
endif(NOT GL_FOUND)


#find GLFW
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})


add_definitions(-DLINUX)
add_definitions(-DGL_GLEXT_PROTOTYPES)

include_directories(${EIGEN3_INCLUDE_DIRS})

# define shader path
set(TOMGINE_5_SHADER ${CMAKE_INSTALL_PREFIX}/include/v4r/TomGine5/shader/ CACHE PATH "Path to shader files")
set(TOMGINE_5_SHADER ${TOMGINE_5_SHADER}/)
add_definitions(-DTOMGINE_5_SHADER="${TOMGINE_5_SHADER}")
message( STATUS "TOMGINE_5_SHADER is ${TOMGINE_5_SHADER}")

set(SOURCE_CPP 
	Camera.cpp
        GLContext.cpp
	GLTexture.cpp
	GLFramebuffer.cpp
        GLUtils.cpp
        GLSLProgram.cpp
	GLWindow.cpp
        SceneWindow.cpp
        Scene.cpp
        SceneCam.cpp
        Font.cpp
        Mesh.cpp
        Background.cpp
        Pyramid.cpp
        SceneWindowManaged.cpp
        PointCloud.cpp
        PointCloudProjected.cpp

)

set(SOURCE_H
        Camera.h
        GLContext.h
	GLTexture.h
	GLFramebuffer.h
        GLUtils.h
        GLSLProgram.h
	GLWindow.h
        SceneWindow.h
        Scene.h
        SceneCam.h
        Font.h
        Mesh.h
        Background.h
        Pyramid.h
        SceneWindowManaged.haddSceneObject
        PointCloud.h
        PointCloudProjected.h
)
set(SOURCE_SHADER

  shader/background.vsh
  shader/background.fsh

  shader/diffuse.vsh
  shader/diffuse.fsh

  shader/pyrBlur.vsh
  shader/pyrBlur.fsh

  shader/pyrBlurv.fsh

  shader/pyrBlurh.fsh

  shader/projPointCloudDotted.vsh
  shader/projPointCloudDotted.fsh
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP} ${SOURCE_SHADER})
target_link_libraries(${PROJECT_NAME} ${GL_LIBRARIES} ${OPENCV_LIBRARIES} glfw GLEW X11 xcb rt assimp boost_thread boost_system )#:/usr/lib/libGLEW.so.1.10.0)# i don't get it done better

if(${V4R_BUILD})

  v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")
  
else(${V4R_BUILD})

  install(DIRECTORY DESTINATION include/v4r/${TOMGINE_INCLUDE_NAME})
  install(FILES ${SOURCE_H} DESTINATION include/v4r/${TOMGINE_INCLUDE_NAME})
  install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

endif(${V4R_BUILD})

ENDIF(V4R_TOMGINE_5)
