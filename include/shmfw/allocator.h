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
    typedef bi::allocator<T, SegmentManager> Allocator;
public:


    /** Default constructor
     * @post Alloc::construct
     **/
    Alloc() {
    }
    /** Constructor
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment handler
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     * @see ShmFw::construct
     **/
    Alloc ( const std::string &name, HandlerPtr &shmHdl) {
        if ( construct ( name, shmHdl ) == ERROR ) exit ( 1 );
    }
    /**
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment handler
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     * @see ShmFw::construct
     **/
    int construct ( const std::string &name, HandlerPtr &shmHdl) {
#if __cplusplus > 199711L
        size_t type_hash_code = typeid ( Alloc<T> ).hash_code();
        const char *type_name = typeid ( Alloc<T> ).name();
#else
        size_t type_hash_code = 0;
        const char *type_name = typeid ( Alloc<T> ).name();
#endif
        if ( constructHeader<SharedHeader> ( name, shmHdl, type_name, type_hash_code ) == ERROR ) return ERROR;
        if ( headerLoc.creator ) {
            /// constructing shared data
            try {
                ScopedLock myLock ( pHeaderShm->mutex );
                pHeaderShm->container = ShmFw::Header::CONTAINER_ALLOC;
                Allocator a ( headerLoc.pShmHdl->getShm()->get_segment_manager() );
                pHeaderShm->data = headerLoc.pShmHdl->getShm()->construct<T> ( bi::anonymous_instance ) [1] ( a );
            } catch ( ... ) {
                std::cerr << "Error when constructing shared data" << std::endl;
                return ERROR;
            }
        }
        return OK;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a pointer to the shared object
     * @return ref to shared data
     **/
    T *ptr()  {
        return (T*) pHeaderShm->data.get();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a pointer to the shared object
     * @return ref to shared data
     **/
    const T *ptr() const {
        return (T*) pHeaderShm->data.get();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object
     * @return ref to shared data
     **/
    T &ref() {
        return *ptr();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object
     * @return ref to shared data
     **/
    const T &ref() const {
        return *ptr();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object
     * @return ref to shared data
     **/
    T &operator() () {
        return ref();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object
     * @return ref to shared data
     **/
    const T &operator() () const {
        return ref();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object
     * @return ref to shared data
     **/
    T *operator-> () {
        return ptr();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object
     * @return ref to shared data
     **/
    const T *operator-> () const {
        return ptr();
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns a human readable string to show the context
     * @return string
     **/
    virtual std::string human_readable() const {
        std::stringstream ss;
        ss << name() << " = " << ref();
        return ss.str();
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * destroies the shared memory
     **/
    virtual void destroy() const {
        headerLoc.pShmHdl->getShm()->destroy_ptr ( ptr() );
        Header::destroy();
    };

    /** UNSAVE!! (user have to lock and to update timestamp)
     *  @param o vector for comparison
     **/
    template<typename T1>
    bool operator == (const T1 &o ) const {
	return (ref() == o);
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     *  @param o vector for comparison
     **/
    template<typename T1>
    bool operator != (const T1 &o ) const {
	return (ref() != o);
    }
    
    /**
     * overloads the << and calls the varalible overloades operator
     **/
    friend std::ostream &operator << ( std::ostream &os, const Alloc<T> &o ) {
        return os << o.ref();
    };

    template<class ForwardIt>
    static ForwardIt next ( ForwardIt it, typename std::iterator_traits<ForwardIt>::difference_type n = 1 ) {
        std::advance ( it, n );
        return it;
    }

    template <typename V, typename Element /*= typename V::value_type*/>
    static void resize ( V& vect, size_t newsize, Element const& element ) {
        if ( vect.size() < newsize ) {
            vect.insert (
                vect.end(),
                newsize - vect.size(),
                element
            );
        } else {
            vect.erase (
                next ( vect.begin(), newsize ),
                vect.end()
            );
        }
    }

    template <typename V>
    static void resize ( V& vect, size_t newsize, typename V::allocator_type const& alloc_inst ) {
        resize ( vect, newsize, typename V::value_type ( alloc_inst ) );
    }

    template <typename V>
    static void resize ( V& vect, size_t newsize ) {
        resize ( vect, newsize, typename V::value_type ( vect.get_allocator() ) );
    }
};
};
#endif //SHARED_MEM_CONTAINER_H


