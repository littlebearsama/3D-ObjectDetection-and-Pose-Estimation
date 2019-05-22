SET(V4R_CVEXTENSION 1 CACHE BOOL "Add description here." )
IF(V4R_CVEXTENSION)

set (CVEXTENSIONS_H  
  operator_cv.h 
  coordinates_cv.h
  polygontest_cv.h
  print_cv.h)


INSTALL(FILES ${CVEXTENSIONS_H} DESTINATION include/v4r/cvextensions)
 
ENDIF(V4R_CVEXTENSION)
