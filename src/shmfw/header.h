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


#ifndef SHARED_MEM_HEADER_H
#define SHARED_MEM_HEADER_H

#include <shmfw/handler.h>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/container/scoped_allocator.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/container/scoped_allocator.hpp>
#include <boost/interprocess/managed_mapped_file.hpp>
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

/**
 * converts angle degrees into rad
 * @param deg angle in degrees
 * @return angle in rad
 **/
inline double DEG2RAD(double deg){
  return M_PI * deg / 180.;
}
/**
 * converts angle rad into degrees
 * @param rad angle in rad
 * @return angle in degrees
 **/
inline double RAD2DEG(double rad){
  return 180. * rad / M_PI;
}

namespace bi = boost::interprocess;
namespace bp = boost::posix_time;
namespace bc = boost::container;

typedef bi::scoped_lock<bi::interprocess_mutex> ScopedLock;

template <typename T> using Allocator = bi::allocator<T, bi::managed_shared_memory::segment_manager>;
typedef bc::scoped_allocator_adaptor<bi::allocator<void, bi::managed_mapped_file::segment_manager> >  void_allocator;
typedef Allocator<void> VoidAllocator;
typedef Allocator<char> CharAllocator;
typedef bi::basic_string<char, std::char_traits<char> , CharAllocator > CharString;

class Handler;
typedef std::shared_ptr<Handler> HandlerPtr;

class HeaderShared {
public:
    HeaderShared ( const VoidAllocator &void_alloc );
    bi::offset_ptr<void> data;              /// offest pointer to the data
    uint8_t  container;                     /// container type @see CONTAINER_HEADER, CONTAINER_VARIABLE, CONTAINER_VECTOR, CONTAINER_DEQUE, ...
    size_t type_hash_code;                  /// varaiable type hash code C++ (2011) @see std::type_info::hash_code
    CharString type_name;                   /// varaiable type information @see std::type_info::name
    CharString info_text;                   /// char array for general use @see info_text()
    bool user_flag;                         /// flag for general usage
    uint32_t user_register;                 /// register for general usage
    bp::ptime timestamp;                    /// timestamp to tag the last change of the shared variable
    bi::interprocess_mutex mutex;           /// mutex
    bi::interprocess_mutex condition_mutex; /// mutex used for wait condition calles
    bi::interprocess_condition condition;   /// used for wait and notify condition calles
};

///local header to to manage changes
class HeaderLocal {
public:
    std::string shm_instance_name;      /// name of the shared header in the shared memory segment
    HandlerPtr shm_handler;             /// smart pointer to the shared memory segment header
    bool creator;                       /// ture if this process created the the shared varaible
    bp::ptime timestamp;                /// time stamp of the last local access to this variable
};

/// Common header of all shared memory segments
class Header {
    friend class boost::serialization::access;
public:
    static const int ERROR = 0;
    static const int OK = 1;
    static const int OK_NEW_HEADER = 2;
    static const int OK_USED_EXITING = 3;
    static const size_t SIZE_TYPE_MSG = 0xFF;
    static const int CONTAINER_HEADER   = 0;
    static const int CONTAINER_VARIABLE = 1;
    static const int CONTAINER_VECTOR   = 2;
    static const int CONTAINER_DEQUE    = 3;
    static const int CONTAINER_IMAGE    = 4;
    static const int CONTAINER_ALLOC  = 5;

protected:
    ///header which is placed in the shared memory
protected:
    HeaderShared *header_shared;          /// shared header
    HeaderLocal header_local;             /// local header

    /** Constructor used if you just have a pointer to the shared memory
     * @param shmHeader name of the variable
     * @param shmHdl pointer to the shared memory segment Handler
     * @param name name of the shared object if you know
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     **/
    int findHeader ( const std::string &name, HandlerPtr &shmHdl );

    /** Constructs the shared header
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment Handler
     * @param type_name
     * @param type_hash
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     **/
    int constructHeader ( const std::string &name, HandlerPtr &shmHdl, const char* type_name, size_t type_hash );
private:
    /** sets the tpye_name and type_hash_code and
     * allocates the shared memory for the type name
     * @see std::typeid
     **/
    template <class T>
    void setType() {
        const char *name = typeid ( T ).name();
        size_t hash_code = 0;
#if __cplusplus > 199711L
        hash_code = typeid ( T ).hash_code();
#endif
        setType ( name, hash_code );
    }
    /** sets the tpye_name and type_hash_code
     * @param name a std::typeid::name
     * @see std::hypeid
     **/
    void setType ( const char *name, size_t hash_code );
public:

    /** Default Constructor
     * @post ShmFw::findHeader
     * @post ShmFw::constructHeader
     **/
    Header();
    /** Constructor used if you just have a pointer to the shared memory
     * @param shmHeader name of the variable
     * @param shmHdl pointer to the shared memory segment Handler
     * @param name name of the shared object if you know
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::findHeader
     * @see ShmFw::createSegment
     **/
    Header ( HandlerPtr shmHdl, const std::string &name );
    /** Constructor
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment Handler
     * @param headerSize size which should be allocated to host the shared header
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::constructHeader
     * @see ShmFw::createSegment
     **/
    Header ( const std::string &name, HandlerPtr shmHdl, unsigned int headerSize = 0 );
    /** sets an info text
     **/
    void info_text ( const char* text );
    /** sets an info text
     **/
    void info_text ( const std::string &text );
    /** returns info text
     * @return info
     **/
    std::string info_text() const ;
    /**
     * Shared variable name
     * @return name
     **/
    std::string name() const ;
    /** Tries to lock
     * @return If the thread acquires ownership of the mutex, returns true
     **/
    bool try_lock();
    /** check if the mutex is locked without locking it
     * @return ture if it is locked
     **/
    bool locked();
    /**
     * Waits until the lock was suggessful lock
     **/
    void lock();
    /**
     * Unlook
     **/
    void unlock();
    /**
     * Returns the mutex
     **/
    bi::interprocess_mutex &mutex();
    /**
     * Waits ms to to look the mutex
     * @param ms to wait befor the function returns it the mutex is locked
     * @return If the thread acquires ownership of the mutex, returns true
     **/
    bool timed_lock ( unsigned int ms ) ;
    /**
     * Info string
     * @param type on true it will also print type informaion
     * @return returns info string
     **/
    std::string info_shm ( bool type = false );
    /**
     * Returns the local time stamp
     * @return timestamp of the local header
     **/
    const boost::posix_time::ptime &timestampLocal() const;
    /**
     * Returns the shared time stamp
     * @return timestamp of the shared header
     **/
    const boost::posix_time::ptime &timestampShm() const;
    /**
     * Sets the local time stamp to now
     * Should after you just accessed the image (reading)
     **/
    void updateTimestampLocal();
    /**
     * Sets the shared and local time stamp to now
     **/
    void updateTimestamps();
    /**
     * Should be called after a process changed the context of the shared variable \n
     * The function differs to dataModified() by notitying subsribers
     * it will update all timestamps and send a notify signal
     * @see hasChanged
     **/
    void itHasChanged();
    /**
     * Blocking function waits until itHasChanged
     * @see itHasChanged
     **/
    void wait();
    /**
     * Blocking function waits until itHasChanged or timeout
     * @param ms timeout in ms
     * @see wait
     * @return False if timeout
     */
    bool timed_wait ( unsigned int ms );
    /**
     * Sets the local time stamps to now
     * Should after you just accessed the image (reading)
     * @see updateTimestampLocal
     **/
    void dataProcessed();
    /**
     * Sets the time stamps to now
     * Should after you just accessed the image (reading)
     * @see updateTimestamps
     **/
    void dataModified();
    /**
     * Checks if the context of shared variable has changed by checking the timestamps
     * call dataProcessed or dataModified
     * @return true if a changed happend
     * @post dataModified or dataProcessed
     **/
    bool hasChanged() const;
    /**
     * sets user flag
     * @post dataProcessed
     **/
    void userFlag ( bool value );
    /**
     * returns user flag
     * @return user flag value
     * @post dataProcessed
     **/
    bool userFlag() const;
    /**
     * sets user register
     * @post dataProcessed
     **/
    void userRegister ( uint32_t value );
    /**
     * returns user register
     * @return user register value
     * @post dataProcessed
     **/
    uint32_t userRegister() const;
    /**
     * container type
     * @return the shared memory container type
     * @see CONTAINER_HEADER, CONTAINER_VARIABLE, ...
     **/
    uint16_t container() const;
    /**
     * container type as string
     * @return the shared memory container type
     * @see CONTAINER_HEADER, CONTAINER_VARIABLE, ...
     **/
    std::string containerName() const;
    /** compares the variable type entries
     * @return true on equal
     **/
    template <class T1>
    bool isType () const {
        const char* type_name_in_shm = header_shared->type_name.c_str();
        const char* type_name_request = typeid ( T1 ).name();
        bool result_name = ( strcmp ( type_name_in_shm, type_name_request ) == 0 );
        bool result_hash_code = true;
        
#if __cplusplus > 199711L
        if ( header_shared->type_hash_code != 0 ) {
            size_t hash_code_request = typeid ( T1 ).hash_code();
            result_hash_code = ( header_shared->type_hash_code == hash_code_request );
        }
#endif
        return result_name && result_hash_code;
    }

    /** compares the variable type entries
     * @param header to compare with
     * @return true on equal
     **/
    bool isType ( const Header &header ) const;

    /** returns the variable tpye name
     * @see std::tpyeid.name()
     * @see isType()
     * @return tpyeid.name();
     **/
    const char* type_name() const;
    /** returns the variable tpye hash code
     * @see std::tpyeid.hash_code()
     * @return tpyeid.hash_code() or if the C++ standard is less than 2011
     **/
    size_t type_hash_code() const;
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared header
     * @warning do not use this fnc, it is only for serialization
     * @return ref to shared data
     **/
    HeaderShared &shared_header();
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared header
     * @warning do not use this fnc, it is only for serialization
     * @return ref to shared data
     **/
    const HeaderShared &shared_header() const;
    /**
     * Destroies the shared variable
     **/
    virtual void destroy() const;
};

typedef std::shared_ptr<Header> HeaderPtr;


};
#endif //SHARED_MEM_HEADER_H



