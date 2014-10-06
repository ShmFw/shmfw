
set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)
message(${CURSES_LIBRARY})

add_executable(shmfw-sandbox sandbox.cpp )
target_link_libraries(shmfw-sandbox  ${Boost_LIBRARIES} rt ${CURSES_LIBRARY})
