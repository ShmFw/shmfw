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


#ifndef SHMFW_SERIALIZATION_INTERPROCESS_VECTOR_HPP
#define SHMFW_SERIALIZATION_INTERPROCESS_VECTOR_HPP


#include <boost/serialization/vector.hpp>
#include <boost/interprocess/containers/vector.hpp>

namespace boost { 
namespace serialization {
  
template<class Archive, class U, class Allocator>

inline void save(
    Archive & ar,
    const boost::interprocess::vector<U, Allocator> &t,
    const unsigned int file_version
){
    boost::serialization::stl::save_collection<Archive, boost::interprocess::vector<U, Allocator> >(
        ar, t
    );
}

template<class Archive, class U, class Allocator>
inline void load(
    Archive & ar,
    boost::interprocess::vector<U, Allocator> &t,
    const unsigned int file_version
){
#ifdef BOOST_SERIALIZATION_VECTOR_135_HPP
    if (ar.get_library_version()==boost::archive::library_version_type(5))
    {
      load(ar,t,file_version, boost::is_arithmetic<U>());
      return;
    }
#endif
    boost::serialization::stl::load_collection<
        Archive,
        boost::interprocess::vector<U, Allocator>,
        boost::serialization::stl::archive_input_seq<
            Archive, boost::interprocess::vector<U, Allocator> 
        >,
        boost::serialization::stl::reserve_imp<boost::interprocess::vector<U, Allocator> >
    >(ar, t);
}

// split non-intrusive serialization function member into separate
// non intrusive save/load member functions
template<class Archive, class U, class Allocator>
inline void serialize(
    Archive & ar,
    boost::interprocess::vector<U, Allocator> & t,
    const unsigned int file_version
){
    boost::serialization::split_free(ar, t, file_version);
}

#if ! BOOST_WORKAROUND(BOOST_MSVC, <= 1300)

/////////1/////////2/////////3/////////4/////////5/////////6/////////7/////////8
// vector<bool>
template<class Archive, class Allocator>
inline void save(
    Archive & ar,
    const boost::interprocess::vector<bool, Allocator> &t,
    const unsigned int /* file_version */
){
    // record number of elements
    collection_size_type count (t.size());
    ar << BOOST_SERIALIZATION_NVP(count);
    boost::interprocess::vector<bool>::const_iterator it = t.begin();
    while(count-- > 0){
        bool tb = *it++;
        ar << boost::serialization::make_nvp("item", tb);
    }
}

template<class Archive, class Allocator>
inline void load(
    Archive & ar,
    boost::interprocess::vector<bool, Allocator> &t,
    const unsigned int /* file_version */
){
    // retrieve number of elements
    collection_size_type count;
    ar >> BOOST_SERIALIZATION_NVP(count);
    t.clear();
    while(count-- > 0){
        bool i;
        ar >> boost::serialization::make_nvp("item", i);
        t.push_back(i);
    }
}

// split non-intrusive serialization function member into separate
// non intrusive save/load member functions
template<class Archive, class Allocator>
inline void serialize(
    Archive & ar,
    boost::interprocess::vector<bool, Allocator> & t,
    const unsigned int file_version
){
    boost::serialization::split_free(ar, t, file_version);
}

#endif // BOOST_WORKAROUND

};
};

#endif // SHMFW_SERIALIZATION_INTERPROCESS_VECTOR_HPP