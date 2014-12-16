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

#include "header.h"

#include "shmfw.h"
#include "handler.h"

#include <iomanip>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace ShmFw;

HeaderShared::HeaderShared ( const VoidAllocator &void_alloc )
: container ( 0 )
, type_hash_code ( 0 )
, type_name ( void_alloc )
, info_text ( void_alloc )
, user_flag ( false )
, user_register ( 0 )
, timestamp ( bp::microsec_clock::local_time() ) {}
        
Header::Header() 
: header_shared ( NULL ) {}

Header::Header ( HandlerPtr shmHdl, const std::string &name )
        : header_shared ( NULL ) {
        if ( findHeader ( name, shmHdl ) == ERROR ) exit ( 1 );
    }

Header::Header ( const std::string &name, HandlerPtr shmHdl, unsigned int headerSize)
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
    
    void Header::info_text ( const char* text ) {
        header_shared->info_text = text;
    }
    void Header::info_text ( const std::string &text ) {
        info_text ( text.c_str() );
    }
    std::string Header::info_text() const {
        return header_shared->info_text.c_str();
    }
    std::string Header::name() const {
        return std::string ( header_local.shm_instance_name );
    }
    bool Header::try_lock() {
        return header_shared->mutex.try_lock();
    }
    bool Header::locked() {
        if ( try_lock() ) {
            unlock();
            return false;
        } else {
            return true;
        }
    }
    void Header::lock() {
        return header_shared->mutex.lock();
    }
    void Header::unlock() {
        return header_shared->mutex.unlock();
    }
    bi::interprocess_mutex &Header::mutex() {
        return header_shared->mutex;
    }
    bool Header::timed_lock ( unsigned int ms ) {
        bp::ptime timeout = bp::microsec_clock::universal_time() + bp::milliseconds ( ms );
        return header_shared->mutex.timed_lock ( timeout );
    }
    std::string Header::info_shm ( bool type ) {
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
    const boost::posix_time::ptime &Header::timestampLocal() const {
        return header_local.timestamp;
    }
    const boost::posix_time::ptime &Header::timestampShm() const {
        return header_shared->timestamp;
    }
    void Header::updateTimestampLocal() {
        header_local.timestamp = now();
    }
    void Header::updateTimestamps() {
        header_local.timestamp = now();
        header_shared->timestamp = header_local.timestamp;
    }
    void Header::itHasChanged() {
        dataModified();
        header_shared->condition.notify_all();
    }
    void Header::wait() {
        ScopedLock lock ( header_shared->condition_mutex );
        header_shared->condition.wait ( lock );
    }
    bool Header::timed_wait ( unsigned int ms ) {
        using namespace bp;
        bp::ptime timeout = bp::microsec_clock::universal_time() + bp::milliseconds ( ms );
        ScopedLock lock ( header_shared->condition_mutex );
        return header_shared->condition.timed_wait ( lock, timeout );
    }
    void Header::dataProcessed() {
        updateTimestampLocal();
    }
    void Header::dataModified() {
        updateTimestamps();
    }
    bool Header::hasChanged() const {
        if ( header_local.timestamp == header_shared->timestamp ) return false;
        bp::time_duration d =  header_local.timestamp - header_shared->timestamp;
        bool check = d.is_negative();
        return check;
    }
    void Header::userFlag ( bool value ) {
        header_shared->user_flag = value;
    }
    bool Header::userFlag() const {
        return header_shared->user_flag;
    }
    void Header::userRegister ( uint32_t value ) {
        header_shared->user_register = value;
    }
    uint32_t Header::userRegister() const {
        return header_shared->user_register;
    }
    uint16_t Header::container() const {
        return header_shared->container;
    }
    std::string Header::containerName() const {

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
    std::string Header::human_readable() const {
        return std::string ( "virtual function human_readable() not implemented!" );
    };
    void Header::destroy() const {
        char *p = ( char * ) header_shared;
        header_local.shm_handler->getShm()->destroy_ptr ( p );
    };
    bool Header::isType ( const Header &header ) const {
        bool result_name = ( strcmp ( header_shared->type_name.c_str(), header.header_shared->type_name.c_str() ) == 0 );
        bool result_hash_code = true;
#if __cplusplus > 199711L
        if ( ( header_shared->type_hash_code != 0 ) && ( header.header_shared->type_hash_code !=0 ) ) {
            result_hash_code = ( header_shared->type_hash_code == header.header_shared->type_hash_code );
        }
#endif
        return result_name && result_hash_code;
    }
    const char* Header::type_name() const {
        return header_shared->type_name.c_str();
    }
    size_t Header::type_hash_code() const {
        return header_shared->type_hash_code;
    }
    HeaderShared &Header::shared_header() {
        return *header_shared;
    }
    const HeaderShared &Header::shared_header() const {
        return *header_shared;
    }
    int Header::findHeader ( const std::string &name, HandlerPtr &shmHdl ) {
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
    int Header::constructHeader ( const std::string &name, HandlerPtr &shmHdl, const char* type_name, size_t type_hash ) {
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
                Allocator<HeaderShared> a ( header_local.shm_handler->getShm()->get_segment_manager() );
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
    void Header::setType ( const char *name, size_t hash_code ) {
        header_shared->type_name = name;
        header_shared->type_hash_code = hash_code;
    }