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


#ifndef SHMFW_SERIALIZATION_MRPT_H
#define SHMFW_SERIALIZATION_MRPT_H


#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/bits.h>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_free.hpp>

namespace boost {
namespace serialization {

/// serialize function
template<class Archive> inline  void serialize ( Archive &ar, mrpt::poses::CPoint2D &o, const unsigned int version ) {
    ar & boost::serialization::make_nvp ( "x", o.m_coords[0] );
    ar & boost::serialization::make_nvp ( "y", o.m_coords[1] );
}
template<class Archive> inline  void serialize ( Archive &ar, mrpt::poses::CPoint3D &o, const unsigned int version ) {
    ar & boost::serialization::make_nvp ( "x", o.m_coords[0] );
    ar & boost::serialization::make_nvp ( "y", o.m_coords[1] );
    ar & boost::serialization::make_nvp ( "z", o.m_coords[2] );
}
template <class Archive> void save ( Archive & ar, const mrpt::poses::CPose2D &o, const unsigned int version ) {
    ar & boost::serialization::make_nvp ( "position", (const mrpt::poses::CPoint2D &) o);
    ar & boost::serialization::make_nvp ( "orientation", o.phi());
}
template <class Archive> void load ( Archive & ar, mrpt::poses::CPose2D &o, const unsigned int version ) {
    ar & boost::serialization::make_nvp ( "position", (mrpt::poses::CPoint2D &) o);
    ar & boost::serialization::make_nvp ( "orientation", o.phi());
}
template<class Archive> inline  void serialize ( Archive &ar, mrpt::poses::CPose2D &o, const unsigned int version ) {
    split_free ( ar,o,version );
}


}; // namespace serialization
}; // namespace boost
#endif // SHMFW_SERIALIZATION_MRPT_H
