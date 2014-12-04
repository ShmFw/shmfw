/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2012 Markus Bader

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef LUVC_DEFAULTS_H
#define LUVC_DEFAULTS_H
#include <linux/videodev2.h>

#define DEFAULT_SHOW_CAMERA_IMAGE true
#define DEFAULT_CONVERT_IMAGE_FIRST 3
#define DEFAULT_FRAME_ID "UVC_CAM"
#define DEFAULT_VIDEODEVICE "/dev/video0"
#define DEFAULT_AVIFILENAME ""
#define DEFAULT_FORMAT      V4L2_PIX_FMT_MJPEG
#define DEFAULT_GRABMETHODE 1
#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480
#define DEFAULT_FPS 30.0



#endif // LUVC_DEFAULTS_H
