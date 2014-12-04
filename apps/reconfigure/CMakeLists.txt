SET(APPS_RECONFIGURE 1 CACHE BOOL "shmfw reconfigure" )
IF(APPS_RECONFIGURE )

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(${PREFIX_BIN}reconfigure src/main_reconfigure.cpp )
target_link_libraries(${PREFIX_BIN}reconfigure ${CURSES_LIBRARY})
_install_binary (${PREFIX_BIN}reconfigure)

add_subdirectory(example)
ENDIF(APPS_RECONFIGURE )
