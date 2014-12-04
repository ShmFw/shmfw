SET(APPS_RECONFIGURE 1 CACHE BOOL "shmfw reconfigure" )
IF(APPS_RECONFIGURE )

set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(shmfw-reconfigure src/main_reconfigure.cpp )
target_link_libraries(shmfw-reconfigure  ${Boost_LIBRARIES} rt ${CURSES_LIBRARY})
install (TARGETS shmfw-reconfigure DESTINATION bin)

add_subdirectory(example)
ENDIF(APPS_RECONFIGURE )
