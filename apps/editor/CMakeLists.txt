SET(APPS_EDITOR 1 CACHE BOOL "shmfw editor" )
IF(APPS_EDITOR )


set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(${PREFIX_BIN}editor src/main_editor.cpp )
target_link_libraries(${PREFIX_BIN}editor ${PREFIX_LIB}core  ${PREFIX_LIB}objects  ${PREFIX_LIB}gerneralization ${CURSES_LIBRARY})
_install_binary (${PREFIX_BIN}editor)

ENDIF(APPS_EDITOR )
