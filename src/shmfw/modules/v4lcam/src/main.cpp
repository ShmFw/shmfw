
#include "v4lcam/v4l_cam.h"


int main(int argc, char **argv)
{
  V4LCam cam;
  cam.initCamera();
  usleep(200000);
  exit(0);
}