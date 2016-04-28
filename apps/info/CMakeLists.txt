SET(APPS_INFO 1 CACHE BOOL "shmfw editor" )
IF(APPS_INFO AND Curses_FOUND )


set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(shmfw-info main_info.cpp )
target_link_libraries(shmfw-info ${CURSES_LIBRARY}  shmfw_core)
_install_binary (shmfw-info)

ENDIF( )