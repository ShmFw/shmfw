SET(APPS_RECONFIGURE 1 CACHE BOOL "shmfw reconfigure" )
IF(APPS_RECONFIGURE )

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(shmfw-reconfigure reconfigure.cpp )
target_link_libraries(shmfw-reconfigure ${CURSES_LIBRARY}  ${PREFIX_LIB}core)
_install_binary (shmfw-reconfigure)

add_subdirectory(demo)
ENDIF(APPS_RECONFIGURE )
