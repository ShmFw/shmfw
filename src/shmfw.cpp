/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2012 by Markus Bader <markus.bader@tuwien.ac.at>        *
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

#include <shmfw/shmfw.h>

#include "shmfw/variable.h"
#include "shmfw/serialization/variable.h"
#include "shmfw/serialization/variable.h"
#include "shmfw/serialization/vector.h"
#include "shmfw/serialization/deque.h"
#include "shmfw/vector.h"
#include "shmfw/deque.h"
#include "shmfw/header.h"
#include "shmfw/log.h"

#include "shmfw/handler_object.h"
#include "shmfw/handler_variable.h"
#include "shmfw/handler_vector.h"
#include "shmfw/handler_deque.h"
#include "shmfw/handler_allocator.h"

#include "shmfw/objects/point.h"
#include "shmfw/objects/quaternion.h"
#include "shmfw/objects/pose.h"
#include "shmfw/objects/pose2d.h"
#include "shmfw/objects/pose2d_agv.h"
#include "shmfw/objects/route_segment.h"
#include "shmfw/objects/parameterentry.h"
#include "shmfw/objects/rgb.h"
#include "shmfw/objects/twist.h"
#include "shmfw/objects/model_state.h"
#include "shmfw/objects/agent_state.h"
#include "shmfw/objects/grid_map.h"
#include "shmfw/objects/mrpt.h"
#include "shmfw/objects/eigen.h"

#include "shmfw/objects/points.h"
#include "shmfw/objects/ros/header.h"
#include "shmfw/objects/ros/visualization_marker.h"

using namespace ShmFw;

template class ShmFw::Var<bool>;
template class ShmFw::Var<char>;
template class ShmFw::Var<short>;
template class ShmFw::Var<int>;
template class ShmFw::Var<long>;
template class ShmFw::Var<unsigned char>;
template class ShmFw::Var<unsigned short>;
template class ShmFw::Var<unsigned int>;
template class ShmFw::Var<unsigned long>;
template class ShmFw::Var<float>;
template class ShmFw::Var<double>;

template class ShmFw::Vector<bool>;
template class ShmFw::Vector<char>;
template class ShmFw::Vector<short>;
template class ShmFw::Vector<int>;
template class ShmFw::Vector<long>;
template class ShmFw::Vector<unsigned char>;
template class ShmFw::Vector<unsigned short>;
template class ShmFw::Vector<unsigned int>;
template class ShmFw::Vector<unsigned long>;
template class ShmFw::Vector<float>;
template class ShmFw::Vector<double>;

template class ShmFw::Deque<bool>;
template class ShmFw::Deque<char>;
template class ShmFw::Deque<short>;
template class ShmFw::Deque<int>;
template class ShmFw::Deque<long>;
template class ShmFw::Deque<unsigned char>;
template class ShmFw::Deque<unsigned short>;
template class ShmFw::Deque<unsigned int>;
template class ShmFw::Deque<unsigned long>;
template class ShmFw::Deque<float>;
template class ShmFw::Deque<double>;

template class ShmFw::ParameterEntry<bool>;
template class ShmFw::ParameterEntry<char>;
template class ShmFw::ParameterEntry<short>;
template class ShmFw::ParameterEntry<int>;
template class ShmFw::ParameterEntry<long>;
template class ShmFw::ParameterEntry<unsigned char>;
template class ShmFw::ParameterEntry<unsigned short>;
template class ShmFw::ParameterEntry<unsigned int>;
template class ShmFw::ParameterEntry<unsigned long>;
template class ShmFw::ParameterEntry<float>;
template class ShmFw::ParameterEntry<double>;


HandlerObjectPtr HandlerObject::open ( const std::string &name, HandlerPtr &shmHdl ) {

    if ( shmHdl->findName ( name ) == NULL ) {
        std::cerr << "no shared variable with name: " << name << std::endl;
        throw 0;
    }
    ShmFw::Header shmHeader ( shmHdl, name );

#define RETURN_IF_TYPE_VAR( TYPE ) \
    if(shmHeader.isType<ShmFw::Var< TYPE > >()) {\
      return HandlerObjectPtr( new ShmFw::HandlerVar< TYPE >(name, shmHdl));\
    };
    RETURN_IF_TYPE_VAR ( float );
    RETURN_IF_TYPE_VAR ( double );
    RETURN_IF_TYPE_VAR ( bool );
    RETURN_IF_TYPE_VAR ( char );
    RETURN_IF_TYPE_VAR ( short );
    RETURN_IF_TYPE_VAR ( int );
    RETURN_IF_TYPE_VAR ( long );
    RETURN_IF_TYPE_VAR ( long long );
    RETURN_IF_TYPE_VAR ( unsigned char );
    RETURN_IF_TYPE_VAR ( unsigned short );
    RETURN_IF_TYPE_VAR ( unsigned int );
    RETURN_IF_TYPE_VAR ( unsigned long );
    RETURN_IF_TYPE_VAR ( unsigned long long );
    RETURN_IF_TYPE_VAR ( int8_t );
    RETURN_IF_TYPE_VAR ( int16_t );
    RETURN_IF_TYPE_VAR ( int32_t );
    RETURN_IF_TYPE_VAR ( int64_t );
    RETURN_IF_TYPE_VAR ( uint8_t );
    RETURN_IF_TYPE_VAR ( uint16_t );
    RETURN_IF_TYPE_VAR ( uint32_t );
    RETURN_IF_TYPE_VAR ( uint64_t );
    RETURN_IF_TYPE_VAR ( Vector3<double> );
    RETURN_IF_TYPE_VAR ( Point );
    RETURN_IF_TYPE_VAR ( Point2D );
    RETURN_IF_TYPE_VAR ( Quaternion );
    RETURN_IF_TYPE_VAR ( Pose );
    RETURN_IF_TYPE_VAR ( Pose2D );
    RETURN_IF_TYPE_VAR ( Pose2DAGV );
    RETURN_IF_TYPE_VAR ( RouteSegment );
    RETURN_IF_TYPE_VAR ( Twist );
    RETURN_IF_TYPE_VAR ( Twist2D );
    RETURN_IF_TYPE_VAR ( ModelState );
    RETURN_IF_TYPE_VAR ( AgentState );
    RETURN_IF_TYPE_VAR ( GridMap<uint8_t> );
    RETURN_IF_TYPE_VAR ( GridMap<uint16_t> );
    RETURN_IF_TYPE_VAR ( GridMap<int8_t> );
    RETURN_IF_TYPE_VAR ( GridMap<int16_t> );
    RETURN_IF_TYPE_VAR ( GridMap<float> );
    RETURN_IF_TYPE_VAR ( GridMap<double> );
    RETURN_IF_TYPE_VAR ( boost::posix_time::ptime );
#ifdef MRPT_FOUND
    RETURN_IF_TYPE_VAR ( mrpt::poses::CPoint2D );
    RETURN_IF_TYPE_VAR ( mrpt::poses::CPoint3D );
    RETURN_IF_TYPE_VAR ( mrpt::poses::CPose2D );
    RETURN_IF_TYPE_VAR ( mrpt::poses::CPose3D );
    RETURN_IF_TYPE_VAR ( mrpt::math::CQuaternion<double> );
    RETURN_IF_TYPE_VAR ( mrpt::math::CQuaternion<float> );
#endif
#ifdef EIGEN_FOUND
    RETURN_IF_TYPE_VAR ( Eigen::Vector2d );
    RETURN_IF_TYPE_VAR ( Eigen::Vector3d );
    RETURN_IF_TYPE_VAR ( Eigen::Vector4d );
    RETURN_IF_TYPE_VAR ( Eigen::Matrix3d );
    RETURN_IF_TYPE_VAR ( Eigen::Matrix4d );
    RETURN_IF_TYPE_VAR ( Eigen::Vector2f );
    RETURN_IF_TYPE_VAR ( Eigen::Vector3f );
    RETURN_IF_TYPE_VAR ( Eigen::Vector4f );
    RETURN_IF_TYPE_VAR ( Eigen::Matrix3f );
    RETURN_IF_TYPE_VAR ( Eigen::Matrix4f );
#endif

#define RETURN_IF_TYPE_VECTOR( TYPE ) if(shmHeader.isType<ShmFw::Vector< TYPE > >()) return HandlerObjectPtr( new ShmFw::HandlerVector< TYPE >( name, shmHdl));
    RETURN_IF_TYPE_VECTOR ( float );
    RETURN_IF_TYPE_VECTOR ( double );
    RETURN_IF_TYPE_VECTOR ( bool );
    RETURN_IF_TYPE_VECTOR ( char );
    RETURN_IF_TYPE_VECTOR ( short );
    RETURN_IF_TYPE_VECTOR ( int );
    RETURN_IF_TYPE_VECTOR ( long );
    RETURN_IF_TYPE_VECTOR ( long long );
    RETURN_IF_TYPE_VECTOR ( unsigned char );
    RETURN_IF_TYPE_VECTOR ( unsigned short );
    RETURN_IF_TYPE_VECTOR ( unsigned int );
    RETURN_IF_TYPE_VECTOR ( unsigned long );
    RETURN_IF_TYPE_VECTOR ( unsigned long long );
    RETURN_IF_TYPE_VECTOR ( Vector3<double> );
    RETURN_IF_TYPE_VECTOR ( Point );
    RETURN_IF_TYPE_VECTOR ( Point2D );
    RETURN_IF_TYPE_VECTOR ( Quaternion );
    RETURN_IF_TYPE_VECTOR ( Pose );
    RETURN_IF_TYPE_VECTOR ( Pose2D );
    RETURN_IF_TYPE_VECTOR ( Pose2DAGV );
    RETURN_IF_TYPE_VECTOR ( RouteSegment );
    RETURN_IF_TYPE_VECTOR ( Twist );
    RETURN_IF_TYPE_VECTOR ( ModelState );
    RETURN_IF_TYPE_VECTOR ( AgentState );
    RETURN_IF_TYPE_VECTOR ( GridMap<uint8_t> );
    RETURN_IF_TYPE_VECTOR ( GridMap<uint16_t> );
    RETURN_IF_TYPE_VECTOR ( GridMap<int8_t> );
    RETURN_IF_TYPE_VECTOR ( GridMap<int16_t> );
    RETURN_IF_TYPE_VECTOR ( GridMap<float> );
    RETURN_IF_TYPE_VECTOR ( GridMap<double> );
    RETURN_IF_TYPE_VECTOR ( boost::posix_time::ptime );
#ifdef MRPT_FOUND
    RETURN_IF_TYPE_VECTOR ( mrpt::poses::CPoint2D );
    RETURN_IF_TYPE_VECTOR ( mrpt::poses::CPoint3D );
    RETURN_IF_TYPE_VECTOR ( mrpt::poses::CPose2D );
    RETURN_IF_TYPE_VECTOR ( mrpt::poses::CPose3D );
    RETURN_IF_TYPE_VECTOR ( mrpt::math::CQuaternion<double> );
    RETURN_IF_TYPE_VECTOR ( mrpt::math::CQuaternion<float> );
#endif
#ifdef EIGEN_FOUND
    RETURN_IF_TYPE_VECTOR ( Eigen::Vector2d );
    RETURN_IF_TYPE_VECTOR ( Eigen::Vector3d );
    RETURN_IF_TYPE_VECTOR ( Eigen::Vector4d );
    RETURN_IF_TYPE_VECTOR ( Eigen::Matrix3d );
    RETURN_IF_TYPE_VECTOR ( Eigen::Matrix4d );
    RETURN_IF_TYPE_VECTOR ( Eigen::Vector2f );
    RETURN_IF_TYPE_VECTOR ( Eigen::Vector3f );
    RETURN_IF_TYPE_VECTOR ( Eigen::Vector4f );
    RETURN_IF_TYPE_VECTOR ( Eigen::Matrix3f );
    RETURN_IF_TYPE_VECTOR ( Eigen::Matrix4f );
#endif

#define RETURN_IF_TYPE_DEQUE( TYPE ) if(shmHeader.isType<ShmFw::Deque< TYPE > >()) return HandlerObjectPtr( new ShmFw::HandlerDeque< TYPE >( name, shmHdl));
    RETURN_IF_TYPE_DEQUE ( float );
    RETURN_IF_TYPE_DEQUE ( double );
    RETURN_IF_TYPE_DEQUE ( bool );
    RETURN_IF_TYPE_DEQUE ( char );
    RETURN_IF_TYPE_DEQUE ( short );
    RETURN_IF_TYPE_DEQUE ( int );
    RETURN_IF_TYPE_DEQUE ( long );
    RETURN_IF_TYPE_DEQUE ( long long );
    RETURN_IF_TYPE_DEQUE ( unsigned char );
    RETURN_IF_TYPE_DEQUE ( unsigned short );
    RETURN_IF_TYPE_DEQUE ( unsigned int );
    RETURN_IF_TYPE_DEQUE ( unsigned long );
    RETURN_IF_TYPE_DEQUE ( unsigned long long );
    RETURN_IF_TYPE_DEQUE ( Vector3<double> );
    RETURN_IF_TYPE_DEQUE ( Point );
    RETURN_IF_TYPE_DEQUE ( Point2D );
    RETURN_IF_TYPE_DEQUE ( Quaternion );
    RETURN_IF_TYPE_DEQUE ( Pose );
    RETURN_IF_TYPE_DEQUE ( Pose2D );
    RETURN_IF_TYPE_DEQUE ( Pose2DAGV );
    RETURN_IF_TYPE_DEQUE ( RouteSegment );
    RETURN_IF_TYPE_DEQUE ( Twist );
    RETURN_IF_TYPE_DEQUE ( Twist2D );
    RETURN_IF_TYPE_DEQUE ( ModelState );
    RETURN_IF_TYPE_DEQUE ( AgentState );
    RETURN_IF_TYPE_DEQUE ( GridMap<uint8_t> );
    RETURN_IF_TYPE_DEQUE ( GridMap<uint16_t> );
    RETURN_IF_TYPE_DEQUE ( GridMap<int8_t> );
    RETURN_IF_TYPE_DEQUE ( GridMap<int16_t> );
    RETURN_IF_TYPE_DEQUE ( GridMap<float> );
    RETURN_IF_TYPE_DEQUE ( GridMap<double> );
    RETURN_IF_TYPE_DEQUE ( boost::posix_time::ptime );

#ifdef MRPT_FOUND
    RETURN_IF_TYPE_DEQUE ( mrpt::poses::CPoint2D );
    RETURN_IF_TYPE_DEQUE ( mrpt::poses::CPoint3D );
    RETURN_IF_TYPE_DEQUE ( mrpt::poses::CPose2D );
    RETURN_IF_TYPE_DEQUE ( mrpt::poses::CPose3D );
    RETURN_IF_TYPE_DEQUE ( mrpt::math::CQuaternion<double> );
    RETURN_IF_TYPE_DEQUE ( mrpt::math::CQuaternion<float> );
#endif

#ifdef EIGEN_FOUND
    RETURN_IF_TYPE_DEQUE ( Eigen::Vector2d );
    RETURN_IF_TYPE_DEQUE ( Eigen::Vector3d );
    RETURN_IF_TYPE_DEQUE ( Eigen::Vector4d );
    RETURN_IF_TYPE_DEQUE ( Eigen::Matrix3d );
    RETURN_IF_TYPE_DEQUE ( Eigen::Matrix4d );
    RETURN_IF_TYPE_DEQUE ( Eigen::Vector2f );
    RETURN_IF_TYPE_DEQUE ( Eigen::Vector3f );
    RETURN_IF_TYPE_DEQUE ( Eigen::Vector4f );
    RETURN_IF_TYPE_DEQUE ( Eigen::Matrix3f );
    RETURN_IF_TYPE_DEQUE ( Eigen::Matrix4f );
#endif
    
#define RETURN_IF_TYPE_ALLOC( TYPE ) if(shmHeader.isType<ShmFw::Alloc< TYPE > >()) return HandlerObjectPtr( new ShmFw::HandlerAlloc< TYPE >( name, shmHdl));
 
    RETURN_IF_TYPE_ALLOC ( Points<Allocator> );
    RETURN_IF_TYPE_ALLOC ( ros::Header );
    RETURN_IF_TYPE_ALLOC ( ros::VisualizationMarker );
    RETURN_IF_TYPE_ALLOC ( ros::VisualizationMarkerArray );
    return HandlerObjectPtr();
}

#define CREATE_TYPE_VAR( TYPE ) if(boost::iequals(type, "TYPE")) return HandlerObjectPtr( new HandlerVar< TYPE >(name, shmHdl));

HandlerObjectPtr HandlerObject::create ( const std::string &name, HandlerPtr &shmHdl, const std::string &type ) {

    if ( shmHdl->findName ( name ) != NULL ) {
        std::cerr << "variable exists allready!" << name << std::endl;
        return open ( name, shmHdl );
    }

    CREATE_TYPE_VAR ( float );
    CREATE_TYPE_VAR ( double );
    CREATE_TYPE_VAR ( bool );
    CREATE_TYPE_VAR ( char );
    CREATE_TYPE_VAR ( short );
    CREATE_TYPE_VAR ( int );
    CREATE_TYPE_VAR ( long );
    CREATE_TYPE_VAR ( long long );
    CREATE_TYPE_VAR ( unsigned char );
    CREATE_TYPE_VAR ( unsigned short );
    CREATE_TYPE_VAR ( unsigned int );
    CREATE_TYPE_VAR ( unsigned long );
    CREATE_TYPE_VAR ( unsigned long long );
    CREATE_TYPE_VAR ( Vector3<double> );
    CREATE_TYPE_VAR ( Point );
    CREATE_TYPE_VAR ( Point2D );
    CREATE_TYPE_VAR ( Quaternion );
    CREATE_TYPE_VAR ( Pose );
    CREATE_TYPE_VAR ( Pose2D );
    CREATE_TYPE_VAR ( Pose2DAGV );
    CREATE_TYPE_VAR ( Twist );
    CREATE_TYPE_VAR ( Twist2D );
    CREATE_TYPE_VAR ( ModelState );
    CREATE_TYPE_VAR ( ModelState );
    CREATE_TYPE_VAR ( AgentState );
    CREATE_TYPE_VAR ( GridMap<uint8_t> );
    CREATE_TYPE_VAR ( GridMap<uint16_t> );
    CREATE_TYPE_VAR ( GridMap<int8_t> );
    CREATE_TYPE_VAR ( GridMap<int16_t> );
    CREATE_TYPE_VAR ( GridMap<float> );
    CREATE_TYPE_VAR ( GridMap<double> );
#ifdef MRPT_FOUND
    CREATE_TYPE_VAR ( mrpt::poses::CPoint2D );
    CREATE_TYPE_VAR ( mrpt::poses::CPoint3D );
    CREATE_TYPE_VAR ( mrpt::poses::CPose2D );
    CREATE_TYPE_VAR ( mrpt::poses::CPose3D );
    CREATE_TYPE_VAR ( mrpt::math::CQuaternion<double> );
    CREATE_TYPE_VAR ( mrpt::math::CQuaternion<float> );
#endif

#ifdef EIGEN_FOUND
    CREATE_TYPE_VAR ( Eigen::Vector2d );
    CREATE_TYPE_VAR ( Eigen::Vector3d );
    CREATE_TYPE_VAR ( Eigen::Vector4d );
    CREATE_TYPE_VAR ( Eigen::Matrix3d );
    CREATE_TYPE_VAR ( Eigen::Matrix4d );
    CREATE_TYPE_VAR ( Eigen::Vector2f );
    CREATE_TYPE_VAR ( Eigen::Vector3f );
    CREATE_TYPE_VAR ( Eigen::Vector4f );
    CREATE_TYPE_VAR ( Eigen::Matrix3f );
    CREATE_TYPE_VAR ( Eigen::Matrix4f );
#endif
    return HandlerObjectPtr();
}
