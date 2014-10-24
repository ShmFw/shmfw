SET(SANDBOX 1 CACHE BOOL "sanbox" )
IF(SANDBOX)
FILE (GLOB FOLDERS "${CMAKE_CURRENT_SOURCE_DIR}/*")

foreach(FOLDER ${FOLDERS})
    if(IS_DIRECTORY ${FOLDER})
      if(${FOLDER} MATCHES "(.*)svn") 
      else()
        MESSAGE( STATUS "Add sandbox folder: ${FOLDER}")
        add_subdirectory(${FOLDER})
      endif()
    endif()
endforeach()


ENDIF(SANDBOX)
