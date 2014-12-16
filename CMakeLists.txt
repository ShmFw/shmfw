cmake_minimum_required ( VERSION 2.6 )
project(ShmFw)

include(macros.cmake)

_version(2014 11 30)
_output_prefix()    
_set_output_paths()
_installation_destinations() # must be befor add_subdirectory

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
find_package (Boost 1.40.0 REQUIRED COMPONENTS date_time thread program_options serialization regex filesystem signals system chrono)
link_libraries(${Boost_LIBRARIES} rt)

find_package( MRPT ) 
IF(MRPT_FOUND)
MESSAGE( STATUS "Building MRPT interface.    \n   -->: " ${MRPT_LIBS_INCL_DIR} )
SET(USE_MRPT 1)
add_definitions( -DMRPT_FOUND )
ELSE (MRPT_FOUND)
MESSAGE( STATUS "MRPT not found" )
ENDIF(MRPT_FOUND)

find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )
SET(EIGEN_FOUND 1)
add_definitions( -DEIGEN_FOUND )

FIND_PACKAGE(OpenCV)
IF(OpenCV_FOUND)
add_definitions( -DOpenCV_FOUND )
MESSAGE( STATUS "Building OpenCV interface.  \n   -->: " ${OpenCV_INCLUDE_DIRS})
ELSE (OpenCV_FOUND)
MESSAGE( STATUS "OpenCV not found" )
ENDIF(OpenCV_FOUND)

find_file(MATHGL_HEADER mgl2/qt.h PATHS)
IF(MATHGL_HEADER)
  SET(MATHGL_FOUND 1)
  add_definitions( -DMATHGL_FOUND )
  set(MATHGL_LIBRARIES mgl-qt mgl)
  MESSAGE( STATUS "MATHGL found" )
else()
  MESSAGE( STATUS ${MATHGL_HEADER})
  MESSAGE( STATUS "--> MathGL: Ubuntu 14.04: \"sudo apt-get install mathgl-dev\" ")
endif()


include_directories(src)
add_subdirectory(include)
add_subdirectory(src)
add_subdirectory(apps)
add_subdirectory(samples)
add_subdirectory(sandbox)
add_subdirectory(unittests)


#install routine
#install(DIRECTORY "include/shmfw"  DESTINATION src FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
#add_custom_target(uninstall COMMAND rm -rfv "${CMAKE_INSTALL_PREFIX}/include/shmfw/*" "${CMAKE_INSTALL_PREFIX}/bin/shmfw*" "${CMAKE_INSTALL_PREFIX}/lib/libshmfw*")

_generate_find_package() # must be after add_subdirectory
_uninstall()    
_doxygen()