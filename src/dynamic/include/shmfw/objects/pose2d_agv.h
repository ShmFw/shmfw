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
#ifndef SHARED_MEM_DYNAMIC_OBJECTS_POSE2D_AGV_H
#define SHARED_MEM_DYNAMIC_OBJECTS_POSE2D_AGV_H


#include <shmfw/objects/pose2d.h>

namespace ShmFw {

class Pose2DAGV : public ShmFw::Pose2D {
public:
    int level;
    Pose2DAGV()
        : Pose2D()
        , level ( 0 ) {};
    Pose2DAGV ( const Pose2D &p, int &l )
        : Pose2D ( p )
        , level ( l ) {};

    Pose2DAGV ( const Pose2DAGV &p )
        : Pose2D ( p )
        , level ( p.level ) {};
    std::string getToStringFormat ( const std::string &format ) const {
        char buf[0xFF];
        sprintf ( buf, format.c_str(), position.x, position.y, orientation, level );
        return std::string ( buf );
    }
    std::string getToString() const {
        return getToStringFormat ( "[ [%lf, %lf], [%lf], [%d] ]" );
    }
    bool setFromString ( const std::string &str ) {
        int start = str.find ( "[" );
        int end = str.find_last_of ( "]" );
        std::string data = str.substr ( start+1, end-1 );
        boost::erase_all ( data, " " );
        if ( sscanf ( data.c_str(), "[%lf,%lf],[%lf], [%d]", &position.x, &position.y, &orientation, &level ) == EOF ) return false;
        return true;
    }
    friend std::ostream& operator<< ( std::ostream &output, const Pose2DAGV &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, Pose2DAGV &o ) {
        std::string str;
        getline ( input, str );
        o.setFromString ( str );
        return input;
    }
    bool operator == ( const Pose2DAGV& o ) const {
        return ((Pose2D&) *this) == ((Pose2D&) o) && level == o.level;
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "pose", boost::serialization::base_object<Pose2D> ( *this ) );
        ar & make_nvp ( "level", orientation );
    }
};

};
#endif //SHARED_DYNAMIC_MEM_OBJECTS_POSE2D_AGV_H


