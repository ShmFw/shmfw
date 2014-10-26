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
#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>

/**
 * Namespace for the fast and dynamic shared memory framework
 */
namespace ShmFw {


class HeaderShared {
public:
    HeaderShared ( const VoidAllocator &void_alloc )
        : container ( 0 )
        , type_hash_code ( 0 )
        , type_name ( void_alloc )
        , info_text ( void_alloc )
        , user_flag ( false )
        , user_register ( 0 )
        , timestamp ( bp::microsec_clock::local_time() ) {}
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
    int findHeader ( const std::string &name, HandlerPtr &shmHdl ) {
        header_local.creator = false;
        header_local.shm_handler = shmHdl;
        header_local.shm_instance_name = shmHdl->resolve_namespace ( name );
        try {
            header_shared = ( HeaderShared * ) header_local.shm_handler->getShm()->find<char> ( header_local.shm_instance_name.c_str() ).first;
            if ( header_shared == NULL ) {
                std::cerr << "variable does not exist" << std::endl;
            }
        } catch ( ... ) {
            std::cerr << "Error when constructing shared header" << std::endl;
            return ERROR;
        }
        updateTimestampLocal();
        return OK;
    }

    /** Constructs the shared header
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment Handler
     * @param type_name
     * @param type_hash
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     **/
    int constructHeader ( const std::string &name, HandlerPtr &shmHdl, const char* type_name, size_t type_hash ) {
        typedef bi::allocator<HeaderShared, SegmentManager> Allocator;
        header_shared = NULL;
        header_local.shm_handler = shmHdl;
        //const char* p = pShm->get_device().get_name();
        header_local.shm_instance_name = shmHdl->resolve_namespace ( name );
        /// constructing shared header
        int ret = ERROR;
        try {
            header_shared = header_local.shm_handler->getShm()->find<HeaderShared> ( header_local.shm_instance_name.c_str() ).first;
            if ( header_shared != NULL ) { /// already exists
                header_local.timestamp = header_shared->timestamp;
                header_local.creator = false;
                updateTimestampLocal();
                ret = OK_NEW_HEADER;
            } else {
                Allocator a ( header_local.shm_handler->getShm()->get_segment_manager() );
                header_shared = header_local.shm_handler->getShm()->construct<HeaderShared> ( header_local.shm_instance_name.c_str() ) ( a );
                header_local.creator = true;
                ScopedLock myLock ( header_shared->mutex );
                header_shared->timestamp = bp::microsec_clock::local_time();
                header_local.timestamp = header_shared->timestamp;
                header_shared->condition_mutex.unlock();
                header_shared->condition.notify_all();
                updateTimestamps();
                setType ( type_name, type_hash );
                ret = OK_USED_EXITING;
            }
        } catch ( ... ) {
            std::cerr << "Error when constructing shared header" << std::endl;
        }
        return ret;
    }
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
    void setType ( const char *name, size_t hash_code ) {
        header_shared->type_name = name;
        header_shared->type_hash_code = hash_code;
    }
public:

    /** Default Constructor
     * @post ShmFw::findHeader
     * @post ShmFw::constructHeader
     **/
    Header()
        : header_shared ( NULL ) {
    }
    /** Constructor used if you just have a pointer to the shared memory
     * @param shmHeader name of the variable
     * @param shmHdl pointer to the shared memory segment Handler
     * @param name name of the shared object if you know
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::findHeader
     * @see ShmFw::createSegment
     **/
    Header ( HandlerPtr shmHdl, const std::string &name )
        : header_shared ( NULL ) {
        if ( findHeader ( name, shmHdl ) == ERROR ) exit ( 1 );
    }
    /** Constructor
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment Handler
     * @param headerSize size which should be allocated to host the shared header
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::constructHeader
     * @see ShmFw::createSegment
     **/
    Header ( const std::string &name, HandlerPtr shmHdl, unsigned int headerSize = 0 )
        : header_shared ( NULL ) {
#if __cplusplus > 199711L
        size_t type_hash_code = typeid ( Header ).hash_code();
        const char *type_name = typeid ( Header ).name();
#else
        size_t type_hash_code = 0;
        const char *type_name = typeid ( Header ).name();
#endif
        if ( constructHeader ( name, shmHdl, type_name, type_hash_code ) == ERROR ) exit ( 1 );
    }
    /** sets an info text
     **/
    void info_text ( const char* text ) {
        header_shared->info_text = text;
    }
    /** sets an info text
     **/
    void info_text ( const std::string &text ) {
        info_text ( text.c_str() );
    }
    /** returns info text
     * @return info
     **/
    std::string info_text() const {
        return header_shared->info_text.c_str();
    }
    /**
     * Shared variable name
     * @return name
     **/
    std::string name() const {
        return std::string ( header_local.shm_instance_name );
    }
    /** Tries to lock
     * @return If the thread acquires ownership of the mutex, returns true
     **/
    bool try_lock() {
        return header_shared->mutex.try_lock();
    }
    /** check if the mutex is locked without locking it
     * @return ture if it is locked
     **/
    bool locked() {
        if ( try_lock() ) {
            unlock();
            return false;
        } else {
            return true;
        }
    }
    /**
     * Waits until the lock was suggessful lock
     **/
    void lock() {
        return header_shared->mutex.lock();
    }
    /**
     * Unlook
     **/
    void unlock() {
        return header_shared->mutex.unlock();
    }
    /**
     * Returns the mutex
     **/
    bi::interprocess_mutex &mutex() {
        return header_shared->mutex;
    }
    /**
     * Waits ms to to look the mutex
     * @param ms to wait befor the function returns it the mutex is locked
     * @return If the thread acquires ownership of the mutex, returns true
     **/
    bool timed_lock ( unsigned int ms ) {
        bp::ptime timeout = bp::microsec_clock::universal_time() + bp::milliseconds ( ms );
        return header_shared->mutex.timed_lock ( timeout );
    }
    /**
     * Info string
     * @param type on true it will also print type informaion
     * @return returns info string
     **/
    std::string info_shm ( bool type = false ) {
        std::stringstream ss;
        bool locked = try_lock();
        ss << std::setw ( 20 ) << name() << ": " << header_shared->timestamp;
        ss << " container: " << std::setw ( 10 ) << containerName();
        ss << " locked: " << ( locked ? "NO" : "YES" );
        ss << " hash: "<< std::setw ( 3 ) << header_shared->type_hash_code;
        ss << " type: "<< std::setw ( 0x1F ) << std::string ( header_shared->type_name.c_str() );
        if ( locked ) unlock();
        return ss.str();
    }
    /**
     * Sets the local time stamp to now
     * Should after you just accessed the image (reading)
     **/
    void updateTimestampLocal() {
        header_local.timestamp = now();
    }
    /**
     * Returns the local time stamp
     * @return timestamp of the local header
     **/
    const boost::posix_time::ptime &timestampLocal() const {
        return header_local.timestamp;
    }
    /**
     * Returns the shared time stamp
     * @return timestamp of the shared header
     **/
    const boost::posix_time::ptime &timestampShm() const {
        return header_shared->timestamp;
    }
    /**
     * Sets the shared time stamp to now
     * Should after you just wrote into the shared image (writing)
     * But please consider to use updateTimestamps to set both local and shared
     **/
    void updateTimestampShared() {
        header_shared->timestamp = now();
    }
    /**
     * Sets the shared and local time stamp to now
     **/
    void updateTimestamps() {
        header_local.timestamp = now();
        header_shared->timestamp = header_local.timestamp;
    }
    /**
     * Should be called after a process changed the context of the shared variable \n
     * it will update all timestamps and send a notify signal
     * @see waitForChange
     **/
    void itHasChanged() {
        updateTimestamps();
        header_shared->condition.notify_all();
    }
    /**
     * Blocking function waits until itHasChanged
     * @see itHasChanged
     **/
    void wait() {
        ScopedLock lock ( header_shared->condition_mutex );
        header_shared->condition.wait ( lock );
    }
    /**
     * Blocking function waits until itHasChanged or timeout
     * @param ms timeout in ms
     * @see wait
     * @return False if timeout
     */
    bool timed_wait ( unsigned int ms ) {
        using namespace bp;
        bp::ptime timeout = bp::microsec_clock::universal_time() + bp::milliseconds ( ms );
        ScopedLock lock ( header_shared->condition_mutex );
        return header_shared->condition.timed_wait ( lock, timeout );
    }
    /**
     * Sets the time stamps to now
     * Should after you just accessed the image (reading)
     * @see updateTimestamps
     **/
    void dataProcessed() {
        updateTimestamps();
    }
    /**
     * Checks if the context of shared variable has changed by checking the timestamps
     * @return true if a changed happend
     * @post dataProcessed
     **/
    bool hasChanged() const {
        if ( header_local.timestamp == header_shared->timestamp ) return false;
        bp::time_duration d =  header_local.timestamp - header_shared->timestamp;
        bool check = d.is_negative();
        return check;
    }
    /**
     * sets user flag
     * @post dataProcessed
     **/
    void userFlag ( bool value ) {
        header_shared->user_flag = value;
    }
    /**
     * returns user flag
     * @return user flag value
     * @post dataProcessed
     **/
    bool userFlag() const {
        return header_shared->user_flag;
    }
    /**
     * sets user register
     * @post dataProcessed
     **/
    void userRegister ( uint32_t value ) {
        header_shared->user_register = value;
    }
    /**
     * returns user register
     * @return user register value
     * @post dataProcessed
     **/
    uint32_t userRegister() const {
        return header_shared->user_register;
    }

    /**
     * container type
     * @return the shared memory container type
     * @see CONTAINER_HEADER, CONTAINER_VARIABLE, ...
     **/
    uint16_t container() const {
        return header_shared->container;
    }
    /**
     * container type as string
     * @return the shared memory container type
     * @see CONTAINER_HEADER, CONTAINER_VARIABLE, ...
     **/
    std::string containerName() const {

        switch ( container() ) {
        case ShmFw::Header::CONTAINER_HEADER:
            return "Header";
        case ShmFw::Header::CONTAINER_VARIABLE:
            return "Variable";
        case ShmFw::Header::CONTAINER_VECTOR:
            return "Vector";
        case ShmFw::Header::CONTAINER_DEQUE:
            return "Deque";
        case ShmFw::Header::CONTAINER_IMAGE:
            return "Image";
        case ShmFw::Header::CONTAINER_ALLOC:
            return "Alloc";
        default:
            return "NA";
        }
    }
    /**
     * Returns a human readable string to show the context
     **/
    virtual std::string human_readable() const {
        return std::string ( "virtual function human_readable() not implemented!" );
    };
    /**
     * Destroies the shared variable
     **/
    virtual void destroy() const {
        char *p = ( char * ) header_shared;
        header_local.shm_handler->getShm()->destroy_ptr ( p );
    };
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
            result_hash_code = ( header_shared->type_hash_code == typeid ( T1 ).hash_code() );
        }
#endif
        return result_name && result_hash_code;
    }

    /** compares the variable type entries
     * @param header to compare with
     * @return true on equal
     **/
    bool isType ( const Header &header ) const {
        bool result_name = ( strcmp ( header_shared->type_name.c_str(), header.header_shared->type_name.c_str() ) == 0 );
        bool result_hash_code = true;
#if __cplusplus > 199711L
        if ( ( header_shared->type_hash_code != 0 ) && ( header.header_shared->type_hash_code !=0 ) ) {
            result_hash_code = ( header_shared->type_hash_code == header.header_shared->type_hash_code );
        }
#endif
        return result_name && result_hash_code;
    }

    /** returns the variable tpye name
     * @see std::tpyeid.name()
     * @see isType()
     * @return tpyeid.name();
     **/
    const char* type_name() const {
        return header_shared->type_name.c_str();
    }
    /** returns the variable tpye hash code
     * @see std::tpyeid.hash_code()
     * @return tpyeid.hash_code() or if the C++ standard is less than 2011
     **/
    size_t type_hash_code() const {
        return header_shared->type_hash_code;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared header
     * @warning do not use this fnc, it is only for serialization
     * @return ref to shared data
     **/
    HeaderShared &shared_header() {
        return *header_shared;
    }
    /** UNSAVE!! (user have to lock and to update timestamp)
     * returns a reference to the shared header
     * @warning do not use this fnc, it is only for serialization
     * @return ref to shared data
     **/
    const HeaderShared &shared_header() const {
        return *header_shared;
    }
};

typedef boost::shared_ptr<Header> HeaderPtr;


};
#endif //SHARED_MEM_HEADER_H



