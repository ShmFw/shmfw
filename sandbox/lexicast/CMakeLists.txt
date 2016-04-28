IF(Curses_FOUND )

add_executable(shmfw-sandbox_lexicast main.cpp )
target_link_libraries(shmfw-sandbox_lexicast  ${Boost_LIBRARIES} rt ${CURSES_LIBRARY})

ENDIF(Curses_FOUND )