#create version header for package
configure_file(config.h.in "${CMAKE_CURRENT_BINARY_DIR}/config.h" @ONLY)
set(RELATIVE_INCLUDE_PATH "shmfw") # ToDo shoule be automated
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h" DESTINATION ${INSTALL_INCLUDE_DIR}/${RELATIVE_INCLUDE_PATH})

