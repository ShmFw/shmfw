SET(APPS_ADMIN 1 CACHE BOOL "admin modul" )
IF(APPS_ADMIN )
include_directories(./include)

add_executable(${PREFIX_BIN}admin src/main_admin.cpp )
target_link_libraries(${PREFIX_BIN}admin ${PREFIX_LIB}core  ${PREFIX_LIB}objects  ${PREFIX_LIB}gerneralization )
_install_binary (${PREFIX_BIN}admin)

ENDIF(APPS_ADMIN )

