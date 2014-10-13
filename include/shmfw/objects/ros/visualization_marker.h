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

#ifndef SHARED_MEM_ROS_VISUALIZATION_MARKER_H
#define SHARED_MEM_ROS_VISUALIZATION_MARKER_H

#include <shmfw/objects/pose.h>
#include <shmfw/objects/ros/header.h>
#include <shmfw/objects/rgba.h>

#ifdef INCLUDE_ROS_HEADERS
#include <ros/ros.h>
#include <shmfw/objects/ros/visualization_marker.h>
#endif

namespace bi = boost::interprocess;

namespace ShmFw {
namespace ros {
typedef bi::allocator<ShmFw::Point, SegmentManager> PointAllocator;
typedef bi::vector<ShmFw::Point, PointAllocator > PointVector;
typedef bi::allocator<ShmFw::RGBA, SegmentManager> RGBAAllocator;
typedef bi::vector<ShmFw::RGBA,RGBAAllocator > RGBAVector;

class VisualizationMarker {
public:
    enum Type {
        MARKER_NA=-1,
        MARKER_ARROW=0,
        MARKER_CUBE=1,
        MARKER_SPHERE=2,
        MARKER_CYLINDER=3,
        MARKER_LINE_STRIP=4,
        MARKER_LINE_LIST=5,
        MARKER_CUBE_LIST=6,
        MARKER_SPHERE_LIST=7,
        MARKER_POINTS=8,
        MARKER_TEXT_VIEW_FACING=9,
        MARKER_MESH_RESOURCE=10,
        MARKER_TRIANGLE_LIST=11
    };
    enum Action {
        ACTION_NA=-1,
        ACTION_ADD=0,
        ACTION_MODYFY=0,
        ACTION_DELETE=2
    };

    Header header;
    CharString ns;
    Type type;
    Action action;
    uint32_t id;
    bool valid;
    ShmFw::Pose pose;
    Vector3<double> scale;
    RGBA color;
    double lifetime;
    bool frame_locked;
    PointVector points;
    RGBAVector colors;
    CharString text;
    CharString mesh_resource;
    bool mesh_use_embedded_materials;

    VisualizationMarker ( const VoidAllocator &void_alloc )
        : header ( void_alloc )
        , ns ( void_alloc )
        , type ( MARKER_NA )
        , action ( ACTION_NA )
        , id ( 0 )
        , valid ( false )
        , pose()
        , scale()
        , color()
        , lifetime ( 0 )
        , frame_locked ( false )
        , points ( void_alloc )
        , colors ( void_alloc )
        , text ( void_alloc )
        , mesh_resource ( void_alloc )
        , mesh_use_embedded_materials ( false ) { }

    VisualizationMarker ( const VisualizationMarker &o, const VoidAllocator &void_alloc )
        : header ( o.header, void_alloc )
        , ns ( o.ns, void_alloc )
        , type ( o.type )
        , action ( o.action )
        , id ( o.id )
        , valid ( o.valid )
        , pose ( o.pose )
        , scale ( o.scale )
        , color ( o.color )
        , lifetime ( o.lifetime )
        , frame_locked ( o.frame_locked )
        , points ( o.points, void_alloc )
        , colors ( o.colors, void_alloc )
        , text ( o.text, void_alloc )
        , mesh_resource ( o.mesh_resource, void_alloc )
        , mesh_use_embedded_materials ( o.mesh_use_embedded_materials ) {
    }

    std::string getToString() const {
        std::stringstream ss;
        ss << "[ " << header;
        ss << ", " << ns;
        ss << ", " << type;
        ss << ", " << action;
        ss << ", " << id << "]";
        return ss.str();
    }
    void getFromString ( const std::string &str ) {
    }
    friend std::ostream& operator<< ( std::ostream &output, const VisualizationMarker &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, VisualizationMarker &o ) {
        return input;
    }
    bool operator == ( const VisualizationMarker& o ) const {
        if ( ! ( o.header == header ) ) return false;
        if ( ! ( o.ns.compare ( ns ) == 0 ) ) return false;
        return true;
    }
    VisualizationMarker &operator = ( const VisualizationMarker& o ) {
        copyFrom ( o );
        return *this;
    }
    void clear ( ) {
        header.clear ( );
        ns.clear();
        type = MARKER_NA;
        action = ACTION_NA;
        id = 0;
        valid = false;
        pose.zero();
        scale.zero();
        color.zero();
        lifetime = 0;
        frame_locked = false;
        points.clear ();
        colors.clear ();
        text.clear ();
        mesh_resource.clear ();
        mesh_use_embedded_materials = false;
    }
    void copyTo ( VisualizationMarker& des ) const {
        header.copyTo ( des.header );
        des.ns = ns;
        des.type = type;
        des.action = action;
        des.id = id;
        des.valid = valid;
        pose.copyTo ( des.pose );
        scale.copyTo ( des.scale );
        color.copyTo ( des.color );
        des.lifetime = lifetime;
        des.frame_locked = frame_locked;
        des.points.resize ( points.size() );
        for ( size_t i = 0; i < points.size(); i++ ) points[i].copyTo ( des.points[i] );
        des.colors.resize ( colors.size() );
        for ( size_t i = 0; i < colors.size(); i++ ) colors[i].copyTo ( des.colors[i] );
        des.text = text;
        des.mesh_resource = mesh_resource;
        des.mesh_use_embedded_materials =  mesh_use_embedded_materials;
    }
    VisualizationMarker& copyFrom ( const VisualizationMarker& src ) {
        header.copyFrom ( src.header );
        ns = src.ns;
        type = src.type;
        action = src.action;
        id = src.id;
        valid = src.valid;
        pose.copyFrom ( src.pose );
        scale.copyFrom ( src.scale );
        color.copyFrom ( src.color );
        lifetime = src.lifetime;
        frame_locked = src.frame_locked;
        points.resize ( src.points.size() );
        for ( size_t i = 0; i < points.size(); i++ ) points[i].copyFrom ( src.points[i] );
        colors.resize ( src.colors.size() );
        for ( size_t i = 0; i < colors.size(); i++ ) colors[i].copyFrom ( src.colors[i] );
        text = src.text;
        mesh_resource = src.mesh_resource;
        mesh_use_embedded_materials =  src.mesh_use_embedded_materials;
        return *this;
    }

#ifdef INCLUDE_ROS_HEADERS
    void copyTo ( visualization_msgs::Marker& des ) const {
        header.copyTo ( des.header );
        des.ns = ns.c_str();
        des.type = type;
        des.action = action;
        des.id = id;
        pose.copyTo ( des.pose );
        scale.copyTo ( des.scale );
        color.copyTo ( des.color );
        des.lifetime.fromSec ( lifetime );
        des.frame_locked = frame_locked;
        des.points.resize ( points.size() );
        for ( size_t i = 0; i < points.size(); i++ ) points[i].copyTo ( des.points[i] );
        des.colors.resize ( colors.size() );
        for ( size_t i = 0; i < colors.size(); i++ ) colors[i].copyTo ( des.colors[i] );
        des.text = text.c_str();
        des.mesh_resource = mesh_resource.c_str();
        des.mesh_use_embedded_materials =  mesh_use_embedded_materials;
    }
#endif

    VisualizationMarker& setArrow ( const std::string &_frame_id, unsigned int _id, const std::string &_ns, const ShmFw::RGBA &_color, const ShmFw::Pose &_pose, double _length = 1, double _lifetime = 0 ) {
        clear();
        header.frame_id = _frame_id.c_str();
        ns = _ns.c_str();
        id = _id;
        valid = true;
        color = _color;
        type = MARKER_ARROW;
        action = ACTION_ADD;
        pose = _pose;
        scale.x = _length;
        scale.y = scale.z = scale.x/20;
        lifetime = _lifetime;
        return *this;
    } 
    VisualizationMarker& setArrow ( const std::string &_frame_id, unsigned int _id, const std::string &_ns, const ShmFw::RGBA &_color, const ShmFw::Point &_a, const ShmFw::Point &_b, double _lifetime = 0 ) {
        clear();
        header.frame_id = _frame_id.c_str();
        ns = _ns.c_str();
        id = _id;
        valid = true;
        color = _color;
        type = MARKER_ARROW;
        action = ACTION_ADD;
        points.resize ( 2 );
        _a.copyTo ( points[0] );
        _b.copyTo ( points[1] );
        double length = ( _a.asVector() - _b.asVector() ).norm();
        scale.x = length/20;
        scale.y = length/15;
        lifetime = _lifetime;
        return *this;
    }
    VisualizationMarker& setLineList ( const std::string &_frame_id, unsigned int _id, const std::string &_ns, const ShmFw::RGBA &_color, double _width = -1., double _lifetime = 0 ) {
        clear();
        header.frame_id = _frame_id.c_str();
        ns = _ns.c_str();
        id = _id;
        valid = true;
        color = _color;
        type = MARKER_LINE_LIST;
        action = ACTION_ADD;
        lifetime = _lifetime;
        scale.x = _width;
        return *this;
    }
    VisualizationMarker& addLineListElement ( const ShmFw::Point &a, const ShmFw::Point &b ) {
        if ( type != MARKER_LINE_LIST ) return *this;
        points.push_back ( a );
        points.push_back ( b );
        return *this;
    }
    VisualizationMarker& setLineStrip ( const std::string &_frame_id, unsigned int _id, const std::string &_ns, const ShmFw::RGBA &_color, double _width = 0.1, double _lifetime = 0 ) {
        clear();
        header.frame_id = _frame_id.c_str();
        ns = _ns.c_str();
        id = _id;
        valid = true;
        color = _color;
        type = MARKER_LINE_STRIP;
        action = ACTION_ADD;
        lifetime = _lifetime;
        scale.x = _width;
        return *this;
    }
    VisualizationMarker& addLineStripElement ( const ShmFw::Point &a ) {
        if ( type != MARKER_LINE_STRIP ) return *this;
        points.push_back ( a );
        return *this;
    }
    VisualizationMarker& setPoints ( const std::string &_frame_id, unsigned int _id, const std::string &_ns, const ShmFw::RGBA &_color, double _width = 0.1, double _height = 0.1, double _lifetime = 0 ) {
        clear();
        header.frame_id = _frame_id.c_str();
        ns = _ns.c_str();
        id = _id;
        valid = true;
        color = _color;
        type = MARKER_POINTS;
        action = ACTION_ADD;
        lifetime = _lifetime;
        scale.x = _width;
        scale.y = _height;
        return *this;
    }
    VisualizationMarker& addPointsElement ( const ShmFw::Point &a ) {
        if ( type != MARKER_POINTS ) return *this;
        points.push_back ( a );
        colors.push_back ( color );
        return *this;
    }
    VisualizationMarker& addPointsElement ( const ShmFw::Point &a, const ShmFw::RGBA &_color ) {
        if ( type != MARKER_POINTS ) return *this;
        points.push_back ( a );
        colors.push_back ( _color );
        return *this;
    }
    VisualizationMarker& setText ( const std::string &_frame_id, unsigned int _id, const std::string &_ns, const ShmFw::RGBA &_color, const std::string &_text, const ShmFw::Point &_p, double _height = 0.1, double _lifetime = 0 ) {
        clear();
        header.frame_id = _frame_id.c_str();
        ns = _ns.c_str();
        id = _id;
        valid = true;
        color = _color;
        type = MARKER_TEXT_VIEW_FACING;
        action = ACTION_ADD;
        lifetime = _lifetime;
        text = _text.c_str();
        scale.x = scale.y = scale.z = _height;
        pose.position = _p;
        //points.push_back (_p );
        return *this;
    }
};

typedef bi::allocator<VisualizationMarker, SegmentManager> VisualizationMarkerAllocator;

class VisualizationMarkerArray {
    typedef bi::vector<VisualizationMarker, VisualizationMarkerAllocator > MarkerVector;
public:
    VisualizationMarkerArray ( const VoidAllocator &void_alloc )
        : markers ( void_alloc ) {
    }
    VisualizationMarkerArray ( const VisualizationMarkerArray& o, const VoidAllocator &void_alloc )
        : markers ( o.markers, void_alloc ) {
    }
    MarkerVector markers;

    std::string getToString() const {
        std::stringstream ss;
        for ( size_t i = 0; i < markers.size(); i++ ) {
            ss << ( i==0?"[ ":", " ) << markers[i];
        }
        ss << "]";
        return ss.str();
    }
    void getFromString ( const std::string &str ) {
    }
    friend std::ostream& operator<< ( std::ostream &output, const VisualizationMarkerArray &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, VisualizationMarkerArray &o ) {
        return input;
    }
    void resize ( size_t newsize, VisualizationMarker const& element ) {
        if ( markers.size() < newsize ) {
            markers.insert ( markers.end(), newsize - markers.size(), element );
        } else {
            MarkerVector::iterator it = markers.begin();
            std::advance ( it, newsize );
            markers.erase ( it, markers.end() );
        }
    }
    void resize ( size_t newsize ) {
        resize ( newsize, typename MarkerVector::value_type ( markers.get_allocator() ) );
    }
    VisualizationMarker &add ( const std::string &ns, int id = -1 ) {
        MarkerVector::iterator it;
        for ( it = markers.begin(); it != markers.end(); it++ ) {
            if ( ns.compare ( it->ns.c_str() ) == 0 ) {
		if((id == -1) || (id == (int) it->id)) {
		  return *it;
		} 
            }
        }
        markers.insert ( markers.end(), 1, MarkerVector::value_type ( markers.get_allocator() ) );
        return markers.back();
    }
    VisualizationMarker &operator() ( const std::string &ns, int _id = -1 ) {
        return add ( ns, _id );
    }
};
};
};


#endif //SHARED_MEM_ROS_VISUALIZATION_MARKER_H



