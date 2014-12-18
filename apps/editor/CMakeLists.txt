SET(APPS_EDITOR 1 CACHE BOOL "shmfw editor" )
IF(APPS_EDITOR )


set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(shmfw-editor main_editor.cpp )
target_link_libraries(shmfw-editor shmfw_core  shmfw_objects  shmfw_gerneralization ${CURSES_LIBRARY})
_install_binary (shmfw-editor)

ENDIF(APPS_EDITOR )
