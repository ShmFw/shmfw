SET(APPS_ADMIN 1 CACHE BOOL "admin modul" )
IF(APPS_ADMIN )
include_directories(./include)

add_executable(shmfw-admin main_admin.cpp )
target_link_libraries(shmfw-admin shmfw_core  shmfw_objects  shmfw_gerneralization )
_install_binary (shmfw-admin)

ENDIF(APPS_ADMIN )

