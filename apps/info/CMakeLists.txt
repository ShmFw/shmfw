SET(APPS_INFO 1 CACHE BOOL "shmfw editor" )
IF(APPS_INFO )


set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(${PREFIX_BIN}info src/main_info.cpp )
target_link_libraries(${PREFIX_BIN}info ${CURSES_LIBRARY}  ${PREFIX_LIB}core)
_install_binary (${PREFIX_BIN}info)

ENDIF(APPS_INFO )