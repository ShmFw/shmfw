/***************************************************************************
 *   Software License Agreement (BSD License)                              *  
 *   Copyright (C) 2014 by Markus Bader <markus.bader@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#ifndef SHARED_MEM_FORWARD_DECLARATIONS_H
#define SHARED_MEM_FORWARD_DECLARATIONS_H


namespace ShmFw {
  class Handler;
  template <class> class Var;
  template <class> class Deque;
  template <class> class Vector;
  template<typename T>  class Alloc;
  template <class> class Vec;
  template <class> class Vector2;
  template <class> class Vector3;
  template <class> class Vector4;
  template <class> class Matrix3x3;
  class Image;
  class LaserScan;
  class Points;
  class Marker;
  class Point;
  class Point2D;
  class Pose;
  class Pose2D;
  class Pose2DAGV;
  class Quaternion;
  class Twist;
  class Twist2D;
  class RouteSegment;
  class ModelState;
  class Handler;
  class AgentState;
  template <typename T> class GridMap;
  template <typename T, template<typename...> class Allocator> class DynamicGridMap;
};


#endif //SHARED_MEM_FORWARD_DECLARATIONS_H


