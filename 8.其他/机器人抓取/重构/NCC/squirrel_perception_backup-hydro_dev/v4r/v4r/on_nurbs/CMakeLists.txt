SET(V4R_ON_NURBS 1 CACHE BOOL "NURBS fitting, based on opennurbs." )
IF(V4R_ON_NURBS)

IF(EXT_OPENNURBS)

find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

PROJECT(v4ron_nurbs)

SET(SOURCE_CPP
	closing_boundary.cpp
	fitting_curve_2d_apdm.cpp
	fitting_curve_2d_asdm.cpp
	fitting_curve_2d_atdm.cpp
	fitting_curve_2d_pdm.cpp
	fitting_curve_2d_sdm.cpp
	fitting_curve_2d_tdm.cpp
	fitting_curve_2d.cpp
	fitting_curve_pdm.cpp
  fitting_cylinder_pdm.cpp
  fitting_sphere_pdm.cpp
  #fitting_surface_depth.h
	fitting_surface_pdm.cpp
	fitting_surface_tdm.cpp
	global_optimization_pdm.cpp
	global_optimization_tdm.cpp
  nurbs_tools.cpp
  sparse_mat.cpp
)

SET(SOURCE_H
	closing_boundary.h
	fitting_curve_2d_apdm.h
	fitting_curve_2d_asdm.h
	fitting_curve_2d_atdm.h
	fitting_curve_2d_pdm.h
	fitting_curve_2d_sdm.h
	fitting_curve_2d_tdm.h
	fitting_curve_2d.h
	fitting_curve_pdm.h
	fitting_cylinder_pdm.h
  fitting_sphere_pdm.h
  #fitting_surface_depth.cpp
	fitting_surface_pdm.h
	fitting_surface_tdm.h
	global_optimization_pdm.h
	global_optimization_tdm.h
	nurbs_data.h
	nurbs_solve.h
  nurbs_tools.h
  sparse_mat.h
)

IF(${PCL_FOUND})

  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  SET(SOURCE_CPP ${SOURCE_CPP}
    fitting_surface_im.cpp
    sequential_fitter.cpp
    triangulation.cpp
  )
  SET(SOURCE_H ${SOURCE_H}
    fitting_surface_im.h
    sequential_fitter.h
    triangulation.h
  )

ENDIF()

SET(V4R_ON_NURBS_SUITESPARSE 1 CACHE BOOL "Use libsuitesparse-dev for solving sparse systems of equations (e.g. in surface/on_nurbs)" )
IF(V4R_ON_NURBS_SUITESPARSE)
  set(SOURCE_CPP ${SOURCE_CPP} nurbs_solve_umfpack.cpp)
  set(ON_NURBS_LIBRARIES ${ON_NURBS_LIBRARIES} cholmod umfpack)
ELSE(V4R_ON_NURBS_SUITESPARSE)
  set(SOURCE_CPP ${SOURCE_CPP} nurbs_solve_eigen_less_3_2.cpp)
ENDIF(V4R_ON_NURBS_SUITESPARSE)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} v4ropennurbs ${ON_NURBS_LIBRARIES})
if(${PCL_FOUND})
  target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
endif()
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ELSE(EXT_OPENNURBS)
  MESSAGE(STATUS "********************************************\n   v4r/on_nurbs: missing EXT_OPENNURBS\n   ********************************************\n")
ENDIF(EXT_OPENNURBS)

ENDIF(V4R_ON_NURBS)

