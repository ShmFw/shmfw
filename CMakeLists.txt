cmake_minimum_required ( VERSION 2.6 )
project(ShmFw)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include(PCHSupport.cmake)

MESSAGE( STATUS "CMAKE_BUILD_TYPE:              " ${CMAKE_BUILD_TYPE} )
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -fno-strength-reduce ")
set(CMAKE_CSS_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

if(CMAKE_BUILD_TYPE MATCHES Release)
MESSAGE( STATUS "CMAKE_CSS_FLAGS_RELEASE:       " ${CMAKE_CSS_FLAGS_RELEASE} )
endif(CMAKE_BUILD_TYPE MATCHES Release)

if(CMAKE_BUILD_TYPE MATCHES Debug)
MESSAGE( STATUS "CMAKE_CXX_FLAGS_DEBUG:         " ${CMAKE_CXX_FLAGS_DEBUG} )
endif(CMAKE_BUILD_TYPE MATCHES Debug)


set(Boost_USE_MULTITHREADED ON)
find_package (Boost 1.40.0 REQUIRED COMPONENTS date_time thread program_options serialization regex filesystem signals system)

find_package( MRPT ) 
IF(MRPT_FOUND)
MESSAGE( STATUS "Building MRPT interface.    \n   -->: " ${MRPT_LIBS_INCL_DIR} )
SET(USE_MRPT 1)
add_definitions( -DUSE_MRPT )
ELSE (MRPT_FOUND)
MESSAGE( STATUS "MRPT not found" )
ENDIF(MRPT_FOUND)

find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )
SET(USE_EIGEN 1)
add_definitions( -DUSE_EIGEN )

FIND_PACKAGE(OpenCV)
IF(OpenCV_FOUND)
MESSAGE( STATUS "Building OpenCV interface.  \n   -->: " ${OpenCV_INCLUDE_DIRS})
SET(USE_OpenCV 1)
add_definitions( -DUSE_OpenCV )
ELSE (OpenCV_FOUND)
MESSAGE( STATUS "OpenCV not found" )
ENDIF(OpenCV_FOUND)

SET(LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin")
SET(LIBRARY_INSTALL_PATH "ShmFw/lib")


include_directories(include)
include_directories(src/dynamic/include)
add_subdirectory(src)

#install routine
install(DIRECTORY "include/shmfw"  DESTINATION include FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
add_custom_target(uninstall COMMAND rm -rfv "${CMAKE_INSTALL_PREFIX}/include/shmfw/*" "${CMAKE_INSTALL_PREFIX}/bin/shmfw*" "${CMAKE_INSTALL_PREFIX}/lib/libshmfw*")
