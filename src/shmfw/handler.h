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


#ifndef SHARED_MEM_HANDLER_H
#define SHARED_MEM_HANDLER_H

#include <shmfw/shmfw.h>
#include <stdexcept>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/interprocess_upgradable_mutex.hpp>
#include <boost/interprocess/sync/upgradable_lock.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cassert>

/**
 * Foward declaration for serialization fiend access
 */
namespace boost {
namespace serialization {
class access;
};
};

/**
 * Namespace for the fast and dynamic shared memory framework
 */
namespace ShmFw {
  
namespace bi = boost::interprocess;
namespace bp = boost::posix_time;

template <typename T> using Allocator = bi::allocator<T, bi::managed_shared_memory::segment_manager>;
  
typedef bi::scoped_lock<bi::interprocess_mutex> ScopedLock;
typedef boost::shared_ptr<ScopedLock> ScopedLockPtr;
typedef boost::shared_ptr<bi::managed_shared_memory> ShmPtr;
typedef bi::managed_shared_memory::segment_manager SegmentManager;
typedef bi::allocator<void,   SegmentManager> VoidAllocator;
typedef bi::allocator<char, SegmentManager>   CharAllocator;
typedef boost::shared_ptr<CharAllocator >   CharAllocatorPtr;
typedef bi::basic_string<char, std::char_traits<char> , CharAllocator > CharString;
typedef bi::allocator<CharString, bi::managed_shared_memory::segment_manager> StringAllocator;
typedef boost::shared_ptr<StringAllocator> StringAllocatorPtr;
typedef boost::shared_ptr<std::stringstream> StringStreamPtr;

class Handler;
typedef boost::shared_ptr<Handler> HandlerPtr;

class Handler {
public:
    /**
     * Default construtor creates a shm with default values
     * @post Handler::createSegment
     **/
    Handler();
    /**
     * Construtor creates a shm
     * @param name
     * @param size
     **/
    Handler ( const std::string &name, unsigned int size = DEFAULT_SEGMENT_SIZE() );
    /**
     * rerates a new handler
     * @post Handler::createSegment
     **/
    static HandlerPtr create();
    /**
     * rerates a new handler
     * @param name
     * @param size
     **/
    static HandlerPtr create ( const std::string &name, unsigned int size = DEFAULT_SEGMENT_SIZE() );
    /**
    * @return managed memory
    **/
    ShmPtr getShm();
    /** creates a named shared memory segment with the current name and size
    **/
    void createSegment();
    /** creates a named shared memory segment
    * @param name name of segment to remove
    * @param size size in bytes of the segment
    **/
    void createSegment ( const std::string &name, unsigned int size = DEFAULT_SEGMENT_SIZE() );
    /**
     * @return shm name
     **/
    const std::string &getName() const;
    /**
     * @return shm size
     **/
    unsigned int getSize() const;
    /**
     * @return ture if the shm exists
     **/
    bool isValid();
    /**
     * used to create a anonymous shared string
     * @param pShm pointer to the shared memory segment
     * @see ShmFw::createSegment
     * @return shared string
     **/
    CharString createString() {
        return CharString ( *pCharAllocator_ );
    }
    /**
     * crates a anonymous string with a context
     * @param str context for the shared string
     * @param pShm pointer to the shared memory segment
     * @see ShmFw::createSegment
     * @return shared string
     **/
    CharString createString ( const char *str ) {
        CharString ret = createString();
        ret = str;
        return ret;
    }
    /**
     * crates a anonymous string with a context
     * @param str context for the shared string
     * @param pShm pointer to the shared memory segment
     * @see ShmFw::createSegment
     * @return shared string
     **/
    CharString createString ( const std::string &str ) {
        CharString ret = createString();
        ret = str.c_str();
        return ret;
    }
    /** Returns list with the names of the shared variables
    * @param rNames vector which will be filled with the variables
    * @param list_hidden on true it will also list names starting with a . 
    * @author Markus Bader
    **/
    void listNames ( std::vector< std::string> &rNames, bool list_hidden );
    /** Returns list with the names of the shared variables
    * @param rName search name
    * @return pointer ot the shared variable or NULL if it was not found
    * @author Markus Bader
    */
    void *findName ( const std::string &rNames, const std::string &prefix = "" );
    /** removes a named shared memory segment
    * @see bi
    * @param name name of segment to remove
    **/
    static bool removeSegment ( const std::string &name );
    /** removes this related shared memory segment
    * @see bi
    **/
    bool removeSegment();

    /** returns namespace
    * @return namespace
    **/
    const std::string &getNamespace() const;
    /** Defines a namespace for shared memory varaibles constructed with this handler
    * a "/" will allways present after calling this fnc
    * @param namespace name prefix
    **/
    void setNamespace ( const std::string& ns );
    /** adds the prefix to a name
    * @param name
    * @return prefix + name
    **/
    std::string resolve_namespace ( const std::string &_name ) ;
private:
    Handler ( const Handler & ) {};
    bool valid_;
    std::string namespace_;
    std::string name_;
    unsigned int size_;
    ShmPtr pShm_;
    CharAllocatorPtr pCharAllocator_;
    StringAllocatorPtr pStringAllocator_;
};


};


#endif //SHARED_MEM_HANDLER_H

