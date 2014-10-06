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

#ifndef SHARED_MEM_CLASS_H
#define SHARED_MEM_CLASS_H

#include <shmfw/header.h>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

/**
 * Namespace for the fast and dynamic shared memory framework
 */
namespace ShmFw {

/// Exdented header (not used in our case)
class SharedHeaderClass : private SharedHeader {
    friend class boost::serialization::access;
};

/// Class to manage a simple shared memory variable or array
template<typename T>
class Class : public Header {
    typedef bi::allocator<T, SegmentManager> Allocator;
    friend class boost::serialization::access;
    SharedHeaderClass *header_shm;  /// exdented shared Header
    LocalData<T>  data_local;         /// local data
public:


    /** Default constructor
     * @post Class::construct
     **/
    Class() {
    }
    /** Constructor
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment handler
     * @param size allows the cration of an array
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     * @see ShmFw::construct
     **/
    Class ( const std::string &name, HandlerPtr &shmHdl ) {
        if ( construct ( name, shmHdl ) == ERROR ) exit ( 1 );
    }
    /**
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment handler
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     * @see ShmFw::construct
     **/
    int construct ( const std::string &name, HandlerPtr &shmHdl ) {
#if __cplusplus > 199711L
        size_t type_hash_code = typeid ( Class<T> ).hash_code() ); const char *type_name = typeid ( Class<T> ).name();
#else
        size_t type_hash_code = 0; const char *type_name = typeid ( Class<T> ).name();
#endif
        if ( constructHeader<SharedHeaderClass> ( name, shmHdl, type_name, type_hash_code ) == ERROR ) return ERROR;;
            header_shm = ( SharedHeaderClass * ) pHeaderShm;
            if ( pHeaderShm->array_size > 0 ) {
                data_local.creator = false;
            } else {
                /// constructing shared data
                try {
                    ScopedLock myLock ( pHeaderShm->mutex );
                        pHeaderShm->container = ShmFw::Header::CONTAINER_BOX;
			Allocator a ( headerLoc.pShmHdl->getShm()->get_segment_manager() );
                        pHeaderShm->ptr = headerLoc.pShmHdl->getShm()->construct<T> ( bi::anonymous_instance )(a);
                        data_local.creator = true;
                        pHeaderShm->array_size = 1;
                    } catch ( ... ) {
                        std::cerr << "Error when constructing shared data" << std::endl;
                        return ERROR;
                    }
                }
        data_local.ptr = ( T * ) pHeaderShm->ptr.get();
        return OK;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared header
     * @warning do not use this fnc, it is only for serialization
     * @return ref to shared data
     **/
    SharedHeaderClass &shared_header() {
        return *header_shm;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared header
     * @warning do not use this fnc, it is only for serialization
     * @return ref to shared data
     **/
    const SharedHeaderClass &shared_header() const {
        return *header_shm;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object by index
     * it is faster as [], but does not check the index > size
     * @n index
     * @return ref to shared data
     **/
    T &operator() ( unsigned int n ) const {
        return data_local.ptr[n];
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object
     * @return ref to shared data
     **/
    T &operator() () const {
        return *data_local.ptr;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a pointer to the shared object
     * @return ref to shared data
     **/
    T *ptr() const {
        return data_local.ptr;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object
     * @return ref to shared data
     **/
    T &ref() {
        return *data_local.ptr;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object
     * @return ref to shared data
     **/
    T &ref() const {
        return *data_local.ptr;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared object
     * @return ref to shared data
     **/
    T *operator-> () const {
        return data_local.ptr;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * Returns a human readable string to show the context
     * @return string
     **/
    virtual std::string human_readable() const {
        std::stringstream ss;
        if ( pHeaderShm->array_size > 1 ) {
            ss << name() << " = [";
            for ( unsigned int i = 0; i < pHeaderShm->array_size; i++ ) {
                ss << ( ( i == 0 ) ? " " : ", " ) << std::setw ( 10 ) << data_local.ptr[i];
            }
            ss << "]";
        } else {
            ss << name() << " = " << data_local.ptr[0];
        }
        return ss.str();
    };
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns array size
     * @return number off ellements allocated in shm
     **/
    unsigned int size() const {
        return pHeaderShm->array_size;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * destroies the shared memory
     **/
    virtual void destroy() const {
        headerLoc.pShmHdl->getShm()->destroy_ptr ( data_local.ptr );
        Header::destroy();
    };

    /**
     * overloads the << and calls the varalible overloades operator
     **/
    friend std::ostream &operator << ( std::ostream &os, const Class<T> &o ) {
        return os << *o.data_local.ptr;
    };
};
};
#endif //SHARED_MEM_CLASS_H


