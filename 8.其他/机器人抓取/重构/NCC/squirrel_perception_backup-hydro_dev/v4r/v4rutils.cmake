
set(V4R_LIBRARIES CACHE INTERNAL "v4r library names" FORCE)
set(V4R_LIBRARIES_PC CACHE INTERNAL "v4r libraries names for pc" FORCE)

macro( v4r_add_headers LIBRARY_NAME SOURCE_HEADER )

        string(REGEX REPLACE "v4r" "" V4R_INCLUDE_NAME ${LIBRARY_NAME})

        set(V4R_LIBRARIES "${V4R_LIBRARIES} ${LIBRARY_NAME}" CACHE INTERNAL "v4r library names")
        set(V4R_LIBRARIES_PC "${V4R_LIBRARIES_PC} -l${LIBRARY_NAME}" CACHE INTERNAL "v4r library names")

        install(DIRECTORY DESTINATION include/v4r/${V4R_INCLUDE_NAME})
        install(FILES ${SOURCE_HEADER} DESTINATION include/v4r/${V4R_INCLUDE_NAME})
        #install(TARGETS ${LIBRARY_NAME} LIBRARY DESTINATION lib)

endmacro( v4r_add_headers )

macro( v4r_add_library LIBRARY_NAME SOURCE_HEADER ) 

	string(REGEX REPLACE "v4r" "" V4R_INCLUDE_NAME ${LIBRARY_NAME})
	
	set(V4R_LIBRARIES "${V4R_LIBRARIES} ${LIBRARY_NAME}" CACHE INTERNAL "v4r library names") 
	set(V4R_LIBRARIES_PC "${V4R_LIBRARIES_PC} -l${LIBRARY_NAME}" CACHE INTERNAL "v4r library names") 
	
	install(DIRECTORY DESTINATION include/v4r/${V4R_INCLUDE_NAME})
	install(FILES ${SOURCE_HEADER} DESTINATION include/v4r/${V4R_INCLUDE_NAME})
	install(TARGETS ${LIBRARY_NAME} LIBRARY DESTINATION lib)

endmacro( v4r_add_library ) 


macro( v4r_add_binary BINARY_NAME )

	install(TARGETS ${BINARY_NAME} DESTINATION bin)

endmacro( v4r_add_binary ) 

