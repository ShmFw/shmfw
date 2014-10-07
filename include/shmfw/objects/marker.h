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

#ifndef SHARED_MEM_MARKER_H
#define SHARED_MEM_MARKER_H

#include <shmfw/objects/pose.h>
#include <shmfw/objects/rgba.h>

namespace bi = boost::interprocess;

namespace ShmFw {


class Marker {
    static const int SIZE_NS = 0x1F;
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
        ACTION_ADD=0,
        ACTION_MODYFY=0,
        ACTION_DELETE=2
    };

    char ns[0x1F];
    Type type;
    Action action;
    uint32_t id;
    bool valid;
    ShmFw::Pose pose;
    ShmFw::Point point;
    Vector3<double> scale;
    RGBA color;
    double lifetime;

    Marker ()
        : type ( MARKER_POINTS )
        , action ( ACTION_ADD )
        , id ( 0 )
	, valid(true)
        , pose()
        , point()
        , scale()
        , color()
        , lifetime ( 0 ) {
        sprintf ( ns, "na" );
    }

    Marker ( const Marker &m )
        : type ( m.type )
        , action ( m.action )
        , id ( m.id )
	, valid(m.valid)
        , pose ( m.pose )
        , point ( m.point )
        , scale ( m.scale )
        , color ( m.color )
        , lifetime ( m.lifetime ) {
        strcpy ( ns,m.ns );
    }

    std::string getToString() const {
        std::stringstream ss;
        ss << id << ": " << (valid?"  valid, ":"invalid, ");
        if ( type == MARKER_NA ) ss << "NA ";
        if ( type == MARKER_POINTS ) ss << "POINTS ";
        if ( type == MARKER_ARROW ) ss << "MARKER_ARROW ";
        if ( type == MARKER_ARROW ) ss << pose << "; " << point;
        return ss.str();
    }
    void setNS ( const std::string &str ) {
        if ( str.length() < SIZE_NS-1 ) {
            sprintf ( ns, "%s", str.c_str() );
        }
    }
    std::string getNS () const {
        return std::string ( ns );
    }
    void getFromString ( const std::string &str ) {
    }

    friend std::ostream& operator<< ( std::ostream &output, const Marker &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, Marker &o ) {
        return input;
    }
    bool operator == ( const Marker& o ) const {
        if ( ! ( type == o.type ) ) return false;
        if ( ! ( action == o.action ) ) return false;
        if ( ! ( valid == o.valid ) ) return false;
        if ( ! ( id == o.id ) ) return false;
        if ( ! ( pose == o.pose ) ) return false;
        if ( ! ( point == o.point ) ) return false;
        if ( ! ( scale == o.scale ) ) return false;
        if ( ! ( color == o.color ) ) return false;
        if ( ! ( lifetime == o.lifetime ) ) return false;
        return true;
    }
    template<typename T>
    void copyTo ( T& des ) const {
        des.type = type;
        des.action = action;
        des.id = id;
        des.valid = valid;
        des.pose = pose;
        des.point = point;
        des.scale = scale;
        des.color = color;
        des.lifetime = lifetime;
    }
    template<typename T>
    Marker& copyFrom ( const T& src ) {
        type = src.type;
        action = src.action;
        id = src.id;
        valid = src.valid;
        pose = src.pose;
        point = src.point;
        scale = src.scale;
        color = src.color;
        lifetime = src.lifetime;
        return *this;
    }
    static const Marker getArrow ( unsigned int id, const std::string &name, const ShmFw::RGBA &color, const ShmFw::Pose &pose, double length = 1, double lifetime = 0 ) {
        ShmFw::Marker m;
        m.setNS ( name );
        m.id = id;
        m.valid = true;
        m.color = color;
        m.type = ShmFw::Marker::MARKER_ARROW;
        m.pose = pose;
        m.scale.x = length;
        m.scale.y = m.scale.z = m.scale.x/20;
	m.lifetime = lifetime;
        return m;
    }
};

};


#endif //SHARED_MEM_MARKER_H


