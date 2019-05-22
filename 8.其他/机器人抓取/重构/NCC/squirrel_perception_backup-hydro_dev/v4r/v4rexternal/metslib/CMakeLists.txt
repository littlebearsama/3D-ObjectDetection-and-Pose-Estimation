SET(EXT_METSLIB 1 CACHE BOOL "Metaheuristics library" )
IF(EXT_METSLIB)

PROJECT(v4rmetslib)

SET(SOURCE_H
  abstract-search.hh
  local-search.hh
  mets.hh
  metslib_config.hh
  model.hh
  observer.hh
  simulated-annealing.hh
  tabu-search.hh
  termination-criteria.hh
)

#add_library(${PROJECT_NAME} SHARED ${SOURCE_H})
#target_link_libraries(${PROJECT_NAME})
v4r_add_headers(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(EXT_METSLIB)

