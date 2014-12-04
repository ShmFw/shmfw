# IMPORTANT: FILES_H must be set under ""
macro( _install_library LIBRARY_NAME FILES_H HEADER_DESTINATION ) 
  set_target_properties(${LIBRARY_NAME} PROPERTIES PUBLIC_HEADER "${FILES_H}")
  install(TARGETS ${LIBRARY_NAME}
    # IMPORTANT: Add the library to the "export-set"
    EXPORT ${PROJECT_NAME}Targets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
    PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/${HEADER_DESTINATION}"
    COMPONENT dev)
  set(EXPORT_LIBS "${EXPORT_LIBS} ${LIBRARY_NAME}" CACHE INTERNAL "libraries to export") 
endmacro( _install_library ) 

macro( _install_binary BINARY_NAME)
  install (TARGETS ${BINARY_NAME} DESTINATION ${INSTALL_BIN_DIR})
endmacro( _install_binary)

macro( _doxygen)
  # add a target to generate API documentation with Doxygen
  find_package(Doxygen)
  if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc
      ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
  endif()
endmacro( _doxygen)


macro( _installation_destinations)
  # Offer the user the choice of overriding the installation directories
  set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
  set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
  set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
  if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKE_DIR CMake)
  else()
    set(DEF_INSTALL_CMAKE_DIR lib/CMake/${PROJECT_NAME})
  endif()
  set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH  "Installation directory for CMake files")

endmacro( _installation_destinations)

macro( _uninstall)
  # uninstall target
  configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
      IMMEDIATE @ONLY)
  add_custom_target(uninstall
      COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endmacro( _uninstall)


macro( _generate_find_package)

  # Make relative paths absolute (needed later on)
  foreach(p LIB BIN INCLUDE CMAKE)
    set(var INSTALL_${p}_DIR)
    if(NOT IS_ABSOLUTE "${${var}}")
      set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
    endif()
  endforeach()


  # Add all targets to the build-tree export set
  foreach(val ${EXPORT_LIBS})
    export(TARGETS ${EXPORT_LIB}  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")
  endforeach()

  
  # Export the package for use from the build-tree
  # (this registers the build-tree with a global CMake-registry)
  export(PACKAGE ${PROJECT_NAME})
  
  # Create the ${PROJECT_NAME}Config.cmake and ${PROJECT_NAME}ConfigVersion files
  file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"  "${INSTALL_INCLUDE_DIR}")
  # ... for the build tree
  set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
  configure_file(Config.cmake.in "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
  # ... for the install tree
  set(CONF_INCLUDE_DIRS "\${${PROJECT_NAME}_CMAKE_DIR}/${REL_INCLUDE_DIR}")
  configure_file(Config.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY)
  # ... for both
  configure_file(ConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)
  
  # Install the Config.cmake and ConfigVersion.cmake
  install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

endmacro( _generate_find_package)

macro( _output_prefix)
  #prefixes for libs and binaries
  string(TOLOWER "${PROJECT_NAME}_" PREFIX_LIB)
  string(TOLOWER "${PROJECT_NAME}-" PREFIX_BIN)
endmacro( _output_prefix)

macro( _set_output_paths)
  # place executeables into a bin folder
  SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin")
  # place libraries into a lib folder
  SET(LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
  # reset export libs
  set(EXPORT_LIBS "" CACHE INTERNAL "libraries to export") 
endmacro( _set_output_paths)

macro( _version MAJOR MINOR PATCH)
  set(${PROJECT_NAME}_MAJOR_VERSION ${MAJOR})
  set(${PROJECT_NAME}_MINOR_VERSION ${MINOR})
  set(${PROJECT_NAME}_PATCH_VERSION ${PATCH})
  set(PACKAGE_VERSION ${MAJOR}.${MINOR}.${MINOR})
endmacro( _version)