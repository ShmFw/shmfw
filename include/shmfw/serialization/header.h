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


#ifndef  SHMFW_SERIALIZATION_HEADER_H
#define SHMFW_SERIALIZATION_HEADER_H


#include <shmfw/header.h>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>

namespace boost {
namespace serialization {

/// serialize function
template<class archive> inline  void serialize ( archive &ar, ShmFw::SharedHeader &o, const unsigned int version ) {
    std::string str;
    unsigned int total_size = o.array_size + o.header_size;
    ar & boost::serialization::make_nvp ( "total_size", total_size );
    ar & boost::serialization::make_nvp ( "array_size", o.array_size );
    ar & boost::serialization::make_nvp ( "container", o.container );
    ar & boost::serialization::make_nvp ( "array_size", o.array_size );
    ar & boost::serialization::make_nvp ( "tstamp", o.tstamp );
    ar & boost::serialization::make_nvp ( "type_hash_code", o.type_hash_code );
    if ( archive::is_saving::value ) {
        str = o.type_name.c_str();
        ar & boost::serialization::make_nvp ( "type_name", str );
	str =  o.info_text.c_str();
	ar & boost::serialization::make_nvp ( "info_text", str );
    }
    if ( archive::is_loading::value ) {
        ar & boost::serialization::make_nvp ( "type_name", str );
	o.type_name = str.c_str();
	ar & boost::serialization::make_nvp ( "info_text", str );
	o.info_text = str.c_str();
    }
}

template<class archive> inline  void serialize ( archive &ar, ShmFw::LocalHeader & o, const unsigned int version ) {
    std::string name ( o.varName );
    ar & boost::serialization::make_nvp ( "varName", name );
}


}; // namespace serialization
}; // namespace boost

#endif // SHMFW_SERIALIZATION_HEADER_H
