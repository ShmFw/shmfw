SET(APPS_INFO 1 CACHE BOOL "shmfw editor" )
IF(APPS_INFO )


set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(shmfw-info src/main_info.cpp )
target_link_libraries(shmfw-info ${Boost_LIBRARIES} rt ${CURSES_LIBRARY})
install (TARGETS shmfw-info DESTINATION bin)

ENDIF(APPS_INFO )