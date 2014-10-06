SET(APPS_EDITOR 1 CACHE BOOL "shmfw editor" )
IF(APPS_EDITOR )


set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(shmfw-editor src/main_editor.cpp )
target_link_libraries(shmfw-editor  shmfw ${Boost_LIBRARIES} rt ${CURSES_LIBRARY})
install (TARGETS shmfw-editor DESTINATION bin)

ENDIF(APPS_EDITOR )
