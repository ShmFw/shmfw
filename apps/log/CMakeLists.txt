SET(APPS_LOG 1 CACHE BOOL "log modul" )
IF(APPS_LOG  AND Curses_FOUND)
include_directories(./include)

add_executable(shmfw-log src/main_log.cpp )
target_link_libraries(shmfw-log  ${CURSES_LIBRARY}  shmfw_core)
install (TARGETS shmfw-log DESTINATION bin)

add_subdirectory(example)
ENDIF( )