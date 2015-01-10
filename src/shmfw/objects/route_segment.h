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
#ifndef SHARED_MEM_DYNAMIC_OBJECTS_ROUTE_SEGMENT_H
#define SHARED_MEM_DYNAMIC_OBJECTS_ROUTE_SEGMENT_H


#include <vector>
#include <shmfw/objects/pose.h>

namespace ShmFw {

class RouteSegment : public ShmFw::BaseObject {
public:
    static const uint32_t ID_NA       = -1;
    static const uint8_t TYPE_NA      = 0;
    static const uint8_t TYPE_LINE    = 1;
    static const uint8_t TYPE_ARC     = 2;
    static const uint8_t TYPE_SPIROS  = 3;
    static const uint8_t TYPE_SPLINE  = 4;
    static const uint8_t ORIENTATION_CLOCKWISE  = 0;
    static const uint8_t ORIENTATION_COUNTER_CLOCKWISE  = 1;
    uint32_t id;
    uint8_t type;
    uint8_t orientation;
    Pose start;
    Pose end;
    Pose center;    
    int8_t level;

    RouteSegment()
        : id ( ID_NA ), type ( TYPE_NA ), orientation(ORIENTATION_CLOCKWISE), start(), end(), center(), level ( 0 ) {};
    RouteSegment ( const RouteSegment &p )
        : id ( p.id )
        , type ( p.type )
        , orientation ( p.orientation )
        , start ( p.start )
        , end ( p.end )
        , center ( p.center )
        , level ( p.level ) {};
    RouteSegment ( uint32_t _id,  uint8_t _type, uint8_t _orientation, const Pose2D &_start, const Pose2D &_end, const Pose2D &_center = Pose2D(), int8_t _level = 0 )
        : id ( _id )
        , type ( _type )
        , orientation ( _orientation )
        , start ( _start )
        , end ( _end )
        , center ( _center )
        , level ( _level ) {};
    const char* getToStringType ( ) const {
        switch ( type ) {
        case TYPE_LINE:
            return "Line";
            break;
        case TYPE_ARC:
            if(orientation == ORIENTATION_CLOCKWISE) return "ArcRight";
            if(orientation == ORIENTATION_COUNTER_CLOCKWISE) return "ArcLeft";
            else  return "Arc";
            break;
        case TYPE_SPIROS:
            return "spiros";
            break;
        case TYPE_SPLINE:
            return "spline";
            break;
        case TYPE_NA:
            return "NA";
            break;
        default:
            return "Unkown";
        }
    }
    std::string getToStringFormat ( const std::string &format ) const {
        char buf[0xFF];
        sprintf ( buf, "%3d, %7s, %s, %s, %s, %2d",
                  id,
                  getToStringType(),
                  start.getToString().c_str(),
                  center.getToString().c_str(),
                  end.getToString().c_str(),
                  level );
        return std::string ( buf );
    }
    std::string getToString() const {
        return getToStringFormat ( "[ [%lf, %lf], [%lf] ]" );
    }
    bool setFromString ( const std::string &str ) {
        return true;
    }
    friend std::ostream& operator<< ( std::ostream &output, const RouteSegment &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, RouteSegment &o ) {
        std::string str;
        getline ( input, str );
        o.setFromString ( str );
        return input;
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "id", id );
        ar & make_nvp ( "type", type );
        ar & make_nvp ( "orientation", orientation );
        ar & make_nvp ( "start", start );
        ar & make_nvp ( "end", end );
        ar & make_nvp ( "center", center );
        ar & make_nvp ( "level", level );
    }
};
};
#endif //SHARED_MEM_DYNAMIC_OBJECTS_ROUTE_SEGMENT_H


