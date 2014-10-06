SET(APPS_ADMIN 1 CACHE BOOL "admin modul" )
IF(APPS_ADMIN )
include_directories(./include)

add_executable(shmfw-admin src/main_admin.cpp )
target_link_libraries(shmfw-admin shmfw ${Boost_LIBRARIES} rt)
install (TARGETS shmfw-admin DESTINATION bin)

ENDIF(APPS_ADMIN )

