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

#ifndef SHARED_MEM_CONTAINER_H
#define SHARED_MEM_CONTAINER_H

#include <shmfw/header.h>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

/**
 * Namespace for the fast and dynamic shared memory framework
 */
namespace ShmFw {


/// Class to manage a simple shared memory variable or array
template<typename T>
class Alloc : public Header {
    friend class boost::serialization::access;
    T *data_element;
public:


    /** Default constructor
     * @post Alloc::construct
     **/
    Alloc() : data_element ( NULL ) {
    }
    /** Constructor
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment handler
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     * @see ShmFw::construct
     **/
    Alloc ( const std::string &name, HandlerPtr &shmHdl ) : data_element ( NULL ) {
        if ( construct ( name, shmHdl ) == ERROR ) exit ( 1 );
    }
    /**
     * @param shm_instance_name name of the shared header element in the memory segment
     * @param shmHdl pointer to the shared memory segment handler
     * @param data on NULL the fnc will an anonymous instance otherwise this data will be linked to the shared header
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     * @see ShmFw::construct
     **/
    int construct ( const std::string &shm_instance_name, HandlerPtr &shmHdl, boost::interprocess::offset_ptr<T> data = NULL ) {
#if __cplusplus > 199711L
        size_t type_hash_code = typeid ( Alloc<T> ).hash_code();
        const char *type_name = typeid ( Alloc<T> ).name();
#else
        size_t type_hash_code = 0;
        const char *type_name = typeid ( Alloc<T> ).name();
#endif
        if ( constructHeader ( shm_instance_name, shmHdl, type_name, type_hash_code ) == ERROR ) return ERROR;
        if ( header_local.creator ) {
            /// constructing shared data
            try {
                ScopedLock myLock ( header_shared->mutex );
                header_shared->container = ShmFw::Header::CONTAINER_ALLOC;
                Allocator<T> a ( header_local.shm_handler->getShm()->get_segment_manager() );
                if ( data ) {
                    header_shared->data = data;
                } else {
                    header_shared->data = header_local.shm_handler->getShm()->construct< T > ( bi::anonymous_instance ) ( a );
                }
            } catch ( ... ) {
                std::cerr << "Error when constructing shared data" << std::endl;
                return ERROR;
            }
        }
        data_element = ( T* ) header_shared->data.get();
        return OK;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a pointer to the shared object
     * @return ref to shared data
     **/
    T *get()  {
        return data_element;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a pointer to the shared object
     * @return ref to shared data
     **/
    const T *get() const {
        return data_element;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a pointer to the shared object
     * @return ref to shared data
     **/
    T *operator->() {
        return get();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a pointer to the shared object
     * @return ref to shared data
     **/
    const T *operator->() const {
        return get();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object
     * @return ref to shared data
     **/
    T &operator*() {
        return *get();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object
     * @return ref to shared data
     **/
    const T &operator*() const {
        return *get();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * destroies the shared memory
     **/
    virtual void destroy() const {
        header_local.shm_handler->getShm()->destroy_ptr ( get() );
        Header::destroy();
    };

    /** UNSAVE!! (user have to lock and to update timestamp)
     *  @param o vector for comparison
     **/
    template<typename T1>
    bool operator == ( const T1 &o ) const {
        return ( *get() == o );
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     *  @param o vector for comparison
     **/
    template<typename T1>
    bool operator != ( const T1 &o ) const {
        return ( *get() != o );
    }

    /**
     * overloads the << and calls the varalible overloades operator
     **/
    friend std::ostream &operator << ( std::ostream &os, const Alloc<T> &o ) {
        return os << *o.get();
    };
};

/** Return the nth successor of iterator it. 
 * on C+11 one can use std::next
  **/
template<class ForwardIt>
inline ForwardIt next ( ForwardIt it, typename std::iterator_traits<ForwardIt>::difference_type n = 1 ) {
    std::advance ( it, n );
    return it;
}

/** this fnc can be used to allocate a vector of containers 
  * @see http://stackoverflow.com/questions/26280041/how-to-i-create-a-boost-interprocess-vector-of-interprocess-containers
  **/
template <typename T, typename Allocator, typename Element /*= typename V::value_type*/>
inline void resize ( boost::container::vector<T, Allocator>& vect, size_t newsize, Element const& element ) {
    if ( vect.size() == newsize ) return;
    if ( vect.size() < newsize ) {
        vect.insert (
            vect.end(),
            newsize - vect.size(),
            element
        );
    } else {
        vect.erase (
#if __cplusplus > 199711L
            std::next ( vect.begin(), newsize ),
#else
            next ( vect.begin(), newsize ),
#endif
            vect.end()
        );
    }
}

/** this fnc can be used to allocate a vector of containers 
  * @see http://stackoverflow.com/questions/26280041/how-to-i-create-a-boost-interprocess-vector-of-interprocess-containers
  **/
template <typename T, typename Allocator>
inline void resize ( boost::container::vector<T, Allocator> & vect, size_t newsize, typename boost::container::vector<T, Allocator>::allocator_type const& alloc_inst ) {
    ShmFw::resize ( vect, newsize, typename boost::container::vector<T, Allocator>::value_type ( alloc_inst ) );
}

/** this fnc can be used to allocate a vector of containers 
  * @see http://stackoverflow.com/questions/26280041/how-to-i-create-a-boost-interprocess-vector-of-interprocess-containers
  **/
template <typename T, typename Allocator>
inline void resize ( boost::container::vector<T, Allocator >& vect, size_t newsize ) {
    ShmFw::resize ( vect, newsize, typename boost::container::vector<T, Allocator >::value_type ( vect.get_allocator() ) );
}
};
#endif //SHARED_MEM_CONTAINER_H


