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
 
 