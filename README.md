shmfw
=====

 A fast dynamic framework for interprocess communication (based on boost)

 
INSTALL:
 run:
 mkdir build
 cd build
 cmake ..
 make
 sudo make install
 sudo vim /etc/ld.so.conf.d/shmfw.conf
   --> add the libpath, something like /usr/local/ShmFw/lib/
 ldconfig
 
 
DEVELOPER:
Developer can add the following statement to your there CMakeList.txt 

if (DEFINED ENV{SHMFW})
  include_directories($ENV{SHMFW}/include/)
  include_directories($ENV{SHMFW}/src/dynamic/include)
  FIND_LIBRARY(SHMFW_LIBRARY libshmfw.so PATHS $ENV{SHMFW}/lib )
  MESSAGE( STATUS "ShmFw found: ${SHMFW_LIBRARY}" )
else()
  FIND_LIBRARY(SHMFW_LIBRARY libshmfw.so PATHS /usr/ )
  message("SHMFW NOT defined -> Using system ShmFw if installed: ${SHMFW_LIBRARY}")
endif()
link_libraries(rt ${SHMFW_LIBRARY})