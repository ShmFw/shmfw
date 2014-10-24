
set(CURSES_NEED_NCURSES TRUE)
find_package(Curses REQUIRED)

message(${CURSES_LIBRARY})

add_executable(shmfw-sandbox_lexicast main.cpp )
target_link_libraries(shmfw-sandbox_lexicast  ${Boost_LIBRARIES} rt ${CURSES_LIBRARY})
