SET(EXT_SIFTGPU 1 CACHE BOOL "SiftGPU" )
IF(EXT_SIFTGPU)

PROJECT(SiftGPU)

ADD_DEFINITIONS(-fPIC)
ADD_DEFINITIONS(-march=native)
ADD_DEFINITIONS(-mfpmath=sse)

SET(CMAKE_CXX_FLAGS "-ggdb -O3 -fPIC -std=c++0x -march=native -mno-avx")


SET(SOURCE_CPP
  CLTexImage.cpp
  GLTexImage.cpp
  ProgramGPU.cpp
  ShaderMan.cpp
  SiftPyramid.cpp
  CuTexImage.cpp
  ProgramCG.cpp
  PyramidCL.cpp
  SiftGPU.cpp
  FrameBufferObject.cpp
  ProgramCL.cpp
  PyramidCU.cpp
  SiftMatch.cpp
  GlobalUtil.cpp
  ProgramGLSL.cpp
  PyramidGL.cpp
  SiftMatchCU.cpp
)

SET(SOURCE_H
  CLTexImage.h
  GLTexImage.h
  ProgramCU.h
  PyramidCU.h
  SiftMatchCU.h
  CuTexImage.h
  LiteWindow.h
  ProgramGLSL.h
  PyramidGL.h
  SiftMatch.h
  FrameBufferObject.h
  ProgramCG.h
  ProgramGPU.h
  ShaderMan.h
  SiftPyramid.h
  GlobalUtil.h
  ProgramCL.h
  PyramidCL.h
  SiftGPU.h
)

add_library(siftgpu SHARED ${SOURCE_CPP})
target_link_libraries(siftgpu GLEW glut IL GL X11)

install(DIRECTORY DESTINATION include/v4rexternal/SiftGPU/src/SiftGPU)
install(TARGETS siftgpu LIBRARY DESTINATION lib)
install(FILES ${SOURCE_H} DESTINATION include/v4rexternal/SiftGPU/src/SiftGPU)

#add_subdirectory(demo)

ENDIF(EXT_SIFTGPU)

