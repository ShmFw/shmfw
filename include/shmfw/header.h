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

/*
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/archive_exception.hpp>
*/

#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>

/**
 * Namespace for the fast and dynamic shared memory framework
 */
namespace ShmFw {


class SharedHeader {
public:
    SharedHeader()
        : header_size ( 0 )
        , array_size ( 0 )
        , container ( 0 )
        , type_hash_code ( 0 )
        , user_flag ( false )
        , user_register ( 0 )
        , tstamp ( bp::microsec_clock::local_time() ) {}
    uint32_t header_size;                   /// reseved size for the header which can be bigger as sizeof(SharedHeader)
    uint32_t array_size;                    /// array size, a zero value will mark noninitilized element,
    uint8_t  container;                     /// container type @see CONTAINER_HEADER, CONTAINER_VARIABLE, CONTAINER_VECTOR, CONTAINER_DEQUE, ...
    size_t type_hash_code;                  /// varaiable type hash code C++ (2011) @see std::type_info::hash_code
    bi::offset_ptr<char> type_name;         /// varaiable type information @see std::type_info::name
    bi::offset_ptr<char> info_text;         /// char array for general use @see info_text()
    bool user_flag;                         /// flag for general usage
    uint32_t user_register;                 /// register for general usage
    bp::ptime tstamp;                       /// timestamp to tag the last change of the shared variable
    bi::offset_ptr<void> ptr;               /// offest pointer to the data
    bi::interprocess_mutex mutex;           /// mutex
    bi::interprocess_mutex condition_mutex; /// mutex used for wait condition calles
    bi::interprocess_condition condition;   /// used for wait and notify condition calles
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

///local header to to manage changes
    struct LocalHeader {
        std::string varName;            /// name of the shared varaible
        HandlerPtr pShmHdl;             /// smart pointer to the shared memory segment header
        bool creator;                   /// ture if this process created the the shared varaible
        bp::ptime tstamp;               /// time stamp of the last local access to this variable
    };

/// Local data
    template<typename T>
    struct LocalData {
        bool creator;           /// true if shared memory variable was crated by this process
        T *ptr;                 /// pointer to the shared memory variable
    };

protected:
    ///header which is placed in the shared memory
protected:
    SharedHeader *pHeaderShm;          /// shared header
    LocalHeader headerLoc;             /// local header

    /** Constructor used if you just have a pointer to the shared memory
     * @param shmHeader name of the variable
     * @param shmHdl pointer to the shared memory segment Handler
     * @param name name of the shared object if you know
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     **/
    int findHeader ( const std::string &name, HandlerPtr &shmHdl ) {
        headerLoc.creator = false;
        headerLoc.pShmHdl = shmHdl;
        headerLoc.varName = shmHdl->resolve_namespace(name);
        try {
            pHeaderShm = ( SharedHeader * ) headerLoc.pShmHdl->getShm()->find<char> ( headerLoc.varName.c_str() ).first;
            if ( pHeaderShm == NULL ) {
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
     * @param headerSize size which should be allocated to host the shared header
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     **/
    template<typename T>
    int constructHeader ( const std::string &name, HandlerPtr &shmHdl, const char* type_name, size_t type_hash ) {
        if ( name.length() > 0xFF - 1 ) throw std::runtime_error ( "Shm::Var::create()! name to long" );
        unsigned int headerSize = sizeof ( T );
        pHeaderShm = NULL;
        headerLoc.pShmHdl = shmHdl;
        //const char* p = pShm->get_device().get_name();
        headerLoc.varName = shmHdl->resolve_namespace(name);
        /// constructing shared header
        int ret = ERROR;
        try {
            pHeaderShm = ( SharedHeader * ) headerLoc.pShmHdl->getShm()->find<char> ( headerLoc.varName.c_str() ).first;
            if ( pHeaderShm != NULL ) { /// already exists
                headerLoc.tstamp = pHeaderShm->tstamp;
                headerLoc.creator = false;
                updateTimestampLocal();
                ret = OK_NEW_HEADER;
            } else {
                pHeaderShm = ( SharedHeader * ) headerLoc.pShmHdl->getShm()->construct<T> ( headerLoc.varName.c_str() ) ();
                headerLoc.creator = true;
                ScopedLock myLock ( pHeaderShm->mutex );
                pHeaderShm->tstamp = bp::microsec_clock::local_time();
                headerLoc.tstamp = pHeaderShm->tstamp;
                pHeaderShm->condition_mutex.unlock();
                pHeaderShm->condition.notify_all();
                pHeaderShm->array_size = 0;
                pHeaderShm->header_size = headerSize;
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
        size_t size_typename = strlen ( name );
        try {
            pHeaderShm->type_name = headerLoc.pShmHdl->getShm()->construct<char> ( bi::anonymous_instance ) [size_typename+1]();
        } catch ( ... ) {
            std::cerr << "Error when creating space for type_name" << std::endl;
        }
        strcpy ( pHeaderShm->type_name.get(), name );
        pHeaderShm->type_hash_code = hash_code;
    }
public:

    /** Default Constructor
     * @post ShmFw::findHeader
     * @post ShmFw::constructHeader
     **/
    Header()
        : pHeaderShm ( NULL ) {
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
        : pHeaderShm ( NULL ) {
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
        : pHeaderShm ( NULL ) {
#if __cplusplus > 199711L
        size_t type_hash_code = typeid ( Header ).hash_code(); 
	const char *type_name = typeid ( Header ).name();
#else
        size_t type_hash_code = 0; 
	const char *type_name = typeid ( Header ).name();
#endif
        if ( constructHeader<Header> ( name, shmHdl, type_name, type_hash_code ) == ERROR ) exit ( 1 );
        }
    /** sets an info text
     **/
    void info_text ( const char *text ) {
        size_t size_typename = strlen ( text );
        try {
            if ( pHeaderShm->info_text ) {
                headerLoc.pShmHdl->getShm()->destroy_ptr ( pHeaderShm->info_text.get() );
            }
        } catch ( ... ) {
            std::cerr << "Error when destroyint share space for info string" << std::endl;
        }
        try {
            pHeaderShm->info_text = headerLoc.pShmHdl->getShm()->construct<char> ( bi::anonymous_instance ) [size_typename+1]();
        } catch ( ... ) {
            std::cerr << "Error when creating space for type_name" << std::endl;
        }
        strcpy ( pHeaderShm->type_name.get(), text );
    }
    /** returns info text
     * @return info
     **/
    const char* info_text() {
        return pHeaderShm->info_text.get();
    }
    /**
     * Shared variable name
     * @return name
     **/
    std::string name() const {
        return std::string ( headerLoc.varName );
    }
    /** Tries to lock
     * @return If the thread acquires ownership of the mutex, returns true
     **/
    bool try_lock() {
        return pHeaderShm->mutex.try_lock();
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
        return pHeaderShm->mutex.lock();
    }
    /**
     * Unlook
     **/
    void unlock() {
        return pHeaderShm->mutex.unlock();
    }
    /**
     * Returns the mutex
     **/
    bi::interprocess_mutex &mutex() {
        return pHeaderShm->mutex;
    }
    /**
     * Waits ms to to look the mutex
     * @param ms to wait befor the function returns it the mutex is locked
     * @return If the thread acquires ownership of the mutex, returns true
     **/
    bool timed_lock ( unsigned int ms ) {
        bp::ptime timeout = bp::microsec_clock::universal_time() + bp::milliseconds ( ms );
        return pHeaderShm->mutex.timed_lock ( timeout );
    }
    /**
     * Info string
     * @param type on true it will also print type informaion
     * @return returns info string
     **/
    std::string info ( bool type = false ) {
        std::stringstream ss;
        bool locked = try_lock();
        ss << std::setw ( 20 ) << name() << ": " << pHeaderShm->tstamp;
        ss << " container: " << std::setw ( 10 ) << containerName();
        ss << " locked: " << ( locked ? "NO" : "YES" );
        ss << " hash: "<< std::setw ( 3 ) << pHeaderShm->type_hash_code;
        ss << " type: "<< std::setw ( 0x1F ) << std::string ( pHeaderShm->type_name.get() );
        if ( locked ) unlock();
        return ss.str();
    }
    /**
     * Sets the local time stamp to now
     * Should after you just accessed the image (reading)
     **/
    void updateTimestampLocal() {
        headerLoc.tstamp = now();
    }
    /**
     * Returns the local time stamp
     * @return timestamp of the local header
     **/
    const boost::posix_time::ptime &timestampLocal() const {
        return headerLoc.tstamp;
    }
    /**
     * Returns the shared time stamp
     * @return timestamp of the shared header
     **/
    const boost::posix_time::ptime &timestampShm() const {
        return pHeaderShm->tstamp;
    }
    /**
     * Sets the shared time stamp to now
     * Should after you just wrote into the shared image (writing)
     * But please consider to use updateTimestamps to set both local and shared
     **/
    void updateTimestampShared() {
        pHeaderShm->tstamp = now();
    }
    /**
     * Sets the shared and local time stamp to now
     **/
    void updateTimestamps() {
        headerLoc.tstamp = now();
        pHeaderShm->tstamp = headerLoc.tstamp;
    }
    /**
     * Should be called after a process changed the context of the shared variable \n
     * it will update all timestamps and send a notify signal
     * @see waitForChange
     **/
    void itHasChanged() {
        updateTimestamps();
        pHeaderShm->condition.notify_all();
    }
    /**
     * Blocking function waits until itHasChanged
     * @see itHasChanged
     **/
    void wait() {
        ScopedLock lock ( pHeaderShm->condition_mutex );
        pHeaderShm->condition.wait ( lock );
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
        ScopedLock lock ( pHeaderShm->condition_mutex );
        return pHeaderShm->condition.timed_wait ( lock, timeout );
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
        if ( headerLoc.tstamp == pHeaderShm->tstamp ) return false;
        bp::time_duration d =  headerLoc.tstamp - pHeaderShm->tstamp;
        bool check = d.is_negative();
        return check;
    }
    /**
     * sets user flag
     * @post dataProcessed
     **/
    void userFlag ( bool value ) {
        pHeaderShm->user_flag = value;
    }
    /**
     * returns user flag
     * @return user flag value
     * @post dataProcessed
     **/
    bool userFlag() const {
        return pHeaderShm->user_flag;
    }
    /**
     * sets user register
     * @post dataProcessed
     **/
    void userRegister ( uint32_t value ) {
        pHeaderShm->user_register = value;
    }
    /**
     * returns user register
     * @return user register value
     * @post dataProcessed
     **/
    uint32_t userRegister() const {
        return pHeaderShm->user_register;
    }

    /**
     * container type
     * @return the shared memory container type
     * @see CONTAINER_HEADER, CONTAINER_VARIABLE, ...
     **/
    uint16_t container() const {
        return pHeaderShm->container;
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
        char *p = ( char * ) pHeaderShm;
        headerLoc.pShmHdl->getShm()->destroy_ptr ( p );
        if ( pHeaderShm->info_text ) {
            headerLoc.pShmHdl->getShm()->destroy_ptr ( pHeaderShm->info_text.get() );
        }
        if ( pHeaderShm->type_name ) {
            headerLoc.pShmHdl->getShm()->destroy_ptr ( pHeaderShm->type_name.get() );
        }
    };
    /** compares the variable type entries
     * @return true on equal
     **/
    template <class T1>
    bool isType () const {
        const char* type_name_in_shm = pHeaderShm->type_name.get();
        const char* type_name_request = typeid ( T1 ).name();
        bool result_name = ( strcmp ( type_name_in_shm, type_name_request ) == 0 );
#if __cplusplus > 199711L
        bool result_hash_code = ( pHeaderShm->type_hash_code == typeid ( T1 ).hash_code() );
#else
        bool result_hash_code = true;
#endif
        return result_name && result_hash_code;
    }

    /** compares the variable type entries
     * @param header to compare with
     * @return true on equal
     **/
    bool isType ( const Header &header ) const {
        bool result_name = ( strcmp ( pHeaderShm->type_name.get(), header.pHeaderShm->type_name.get() ) == 0 );
#if __cplusplus > 199711L
        bool result_hash_code = ( pHeaderShm->type_hash_code == header.pHeaderShm->type_hash_code );
#else
        bool result_hash_code = true;
#endif
        return result_name && result_hash_code;
    }

    /** returns the variable tpye name
     * @see std::tpyeid.name()
     * @see isType()
     * @return tpyeid.name();
     **/
    const char* type_name() const {
        return pHeaderShm->type_name.get();
    }
    /** returns the variable tpye hash code
     * @see std::tpyeid.hash_code()
     * @return tpyeid.hash_code() or if the C++ standard is less than 2011
     **/
    size_t type_hash_code() const {
        return pHeaderShm->type_hash_code;
    }
};

typedef boost::shared_ptr<Header> HeaderPtr;


};
#endif //SHARED_MEM_HEADER_H



