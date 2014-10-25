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


#ifndef  SHMFW_SERIALIZATION_VARIABLE_H
#define SHMFW_SERIALIZATION_VARIABLE_H



#include <shmfw/variable.h>
#include <shmfw/serialization/header.h>
#include <boost/serialization/array.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>

namespace boost {
namespace serialization {

/// serialize function
template<class archive, class T> inline  void serialize ( archive &ar, ShmFw::Var< T > &o, const unsigned int version ) {
    ar & boost::serialization::make_nvp ( "SharedHeader", o.shared_header() );
    ar & boost::serialization::make_nvp ( "data", o.ref() );
}

}; // namespace serialization
}; // namespace boost

namespace ShmFw {


/// serialization
template<class T>
inline void serialize ( std::stringstream &ss, const ShmFw::Var<T> &src ) {
    boost::archive::xml_oarchive xml ( ss );
    xml << boost::serialization::make_nvp ( "Variable", src );
}

/// deserialization
template<class T>
inline void deserialize ( std::stringstream &ss, ShmFw::Var<T> &des ) {
    boost::archive::xml_iarchive xml ( ss );
    xml >> boost::serialization::make_nvp ( "Variable", des );
}
};

#endif // SHMFW_SERIALIZATION_VARIABLE_H
