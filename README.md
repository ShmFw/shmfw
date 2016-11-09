ShmFw
=====

 
## PREINSTALL:
### On Ubuntu
``` 
sudo apt-get install cmake
sudo apt-get install libboost-all-dev
sudo apt-get install libopencv-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libncurses5-dev
sudo apt-get install libmgl-dev

``` 
## INSTALL:
``` 
 mkdir build
 cd build
 cmake ..  
 # cmake -D CMAKE_INSTALL_PREFIX=/usr/ ..  # suggested on ubuntu
 # cmake -D CMAKE_BUILD_TYPE=DEBUG ..
 # cmake -D UNITTEST=OFF ..
 make
 sudo make install
``` 


## UNINSTALL:
``` 
sudo make uninstall 
```
 
## UNITTESTING:
### On Ubuntu 14.04/16.04
The ubuntu package management installs only gtest sources, because of this you have to compile gtest first.
http://askubuntu.com/questions/145887/why-no-library-files-installed-for-google-test
``` 
sudo apt-get install libgtest-dev
cd /usr/src/gtest && sudo cmake -DBUILD_SHARED_LIBS=ON . && sudo make && sudo mv libg* /usr/lib/
``` 
## KNOWN ISSUS
### On Ubuntu 16.04
The logging module is not working as well as the grid apps. 
``` 
cmake -DAPPS_GRID=OFF
``` 

### In combination with ROS
There are issus reported if ROS is sourced and the ros-*distribo*-opencv is installed.
You mid get an error on some OpenCV includes.

**Solution:**
combile ShmFw before sourcing ros. You mid have to edit your ~/.bashrc.

## Doxygen
For doxygen run
``` 
make doc
``` 
The documentation will be generated into ./doc

## DEVELOPER:
### find_package
shmfw supports the cmake find_package command use
```
find_package(ShmFw)
```
