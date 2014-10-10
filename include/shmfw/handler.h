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


inline bp::ptime now() {
    return bp::microsec_clock::local_time();
}

enum SerializeFormat {
    FORMAT_NA  = 0,
    FORMAT_XML = 1,
    FORMAT_BIN = 2,
    FORMAT_TXT = 3
};

inline std::string DEFAULT_SEGMENT_NAME() {
    return "ShmFw";
};
inline unsigned int DEFAULT_SEGMENT_SIZE() {
    return 16*1048576; //16MB;
};

class Handler;
typedef boost::shared_ptr<Handler> HandlerPtr;

class Handler {
public:
    /**
     * Default construtor creates a shm with default values
     * @post Handler::createSegment
     **/
    Handler()
        : valid_ ( false ), name_ ( DEFAULT_SEGMENT_NAME() ), size_ ( DEFAULT_SEGMENT_SIZE() ) {
    }
    /**
     * Construtor creates a shm
     * @param name
     * @param size
     **/
    Handler ( const std::string &name, unsigned int size = DEFAULT_SEGMENT_SIZE() )
        : valid_ ( false ), name_ ( name ), size_ ( size ) {
        createSegment();
    }
    /**
     * rerates a new handler
     * @post Handler::createSegment
     **/
    static HandlerPtr create() {
        return HandlerPtr ( new Handler );
    }
    /**
     * rerates a new handler
     * @param name
     * @param size
     **/
    static HandlerPtr create ( const std::string &name, unsigned int size = DEFAULT_SEGMENT_SIZE() ) {
        return HandlerPtr ( new Handler ( name, size ) );
    }
    /**
    * @return managed memory
    **/
    ShmPtr getShm() {
        return pShm_;
    }
    /** creates a named shared memory segment with the current name and size
    **/
    void createSegment() {
        createSegment ( name_, size_ );
    }
    /** creates a named shared memory segment
    * @param name name of segment to remove
    * @param size size in bytes of the segment
    **/
    void createSegment ( const std::string &name, unsigned int size = DEFAULT_SEGMENT_SIZE() ) {
        name_ = name;
        size_ = size;
        try {
            /// look if it allready exists
            pShm_ = ShmPtr ( new bi::managed_shared_memory ( bi::open_only, name_.c_str() ) );
            if ( pShm_->get_size() != size_ ) {
                if ( pShm_->get_size() > size_ ) {
                    std::cerr << "Shared memory segment: " << name_ << " exists but is bigger I will use this one";
                } else {
                    std::cerr << "Shared memory segment: " << name_ << " exists with a samller size that can be a problem: " << pShm_->get_size() << " != " << size_ << std::endl;
                }
            }
        } catch ( ... ) {
            /// it did not exist so I will create it
            try {
                pShm_ = ShmPtr ( new bi::managed_shared_memory ( bi::create_only, name_.c_str(), size_ ) );
                /// Clear the memory
                //std::memset(pShm->get_address(), 0, pShm->get_size());
            } catch ( bi::interprocess_exception &ex ) {
                std::cerr << "Problem on crationg the shared memory!" << std::endl;
                std::cout << ex.what() << std::endl;
                exit ( 1 );
            }
        }
        pCharAllocator_ = CharAllocatorPtr ( new CharAllocator ( pShm_->get_segment_manager() ) );
        pStringAllocator_ = StringAllocatorPtr ( new StringAllocator ( pShm_->get_segment_manager() ) );
        valid_ = true;
    }
    /**
     * @return shm name
     **/
    const std::string &getName() const {
        return name_;
    };
    /**
     * @return shm size
     **/
    unsigned int getSize() const {
        return size_;
    };
    bool isValid() {
        valid_ = false;
        try {
            bi::managed_shared_memory shm ( bi::open_only, name_.c_str() );
            valid_ = true;
        } catch ( bi::interprocess_exception &ex ) {
            std::cerr << "shared memory does not exist!: " << ex.what() << std::endl;
        }
        return valid_;
    }
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
    * @param pTypes type of the variable
    * @param regExpressions examples "(.*)bmp",  "(.*)$"
    * @author Markus Bader
    **/
    void listNames ( std::vector< std::string> &rNames, std::string regx = "" ) {
        try {
            typedef bi::managed_shared_memory::const_named_iterator const_named_it;
            const_named_it named_beg = pShm_->named_begin();
            const_named_it named_end = pShm_->named_end();
            for ( ; named_beg != named_end; ++named_beg ) {
                const bi::managed_shared_memory::char_type *name = named_beg->name();
                if ( regx.empty() ) {
                    rNames.push_back ( name );
                } else {
                    /*
                    static const boost::regex expression ( regx.c_str() );
                    if (boost::regex_match ( name, expression ) ) {
                    rNames.push_back(name);
                    if (pTypes)  pTypes->push_back(pHeader->getVarType());
                    }
                    */
                    std::string entryName ( name );
                    if ( entryName.find ( regx ) != entryName.npos )  {
                        rNames.push_back ( name );
                    }
                }
            }
        }    catch ( ... ) {
            //AK_LOG_ERROR << "exception ShmManager::listNames()";
            std::cerr << "exception ShmManager::listNames()";
        }
    }

    /** Returns list with the names of the shared variables
    * @param rName search name
    * @return pointer ot the shared variable or NULL if it was not found
    * @author Markus Bader
    */
    void *findName ( const std::string &rNames, std::string prefix = "" ) {
        void *value = NULL;
        std::string full_name = "";

        if ( prefix.compare ( "" ) != 0 )
            full_name = prefix + ":" + rNames;
        else
            full_name = rNames;

        try {
            typedef bi::managed_shared_memory::const_named_iterator const_named_it;
            const_named_it named_beg = pShm_->named_begin();
            const_named_it named_end = pShm_->named_end();
            for ( ; named_beg != named_end; ++named_beg ) {
                const bi::managed_shared_memory::char_type *name = named_beg->name();
                if ( full_name.compare ( name ) == 0 ) {
                    value = ( void* ) named_beg->value();
                }
            }
        }    catch ( ... ) {
            //AK_LOG_ERROR << "exception ShmManager::findName(): " << rNames;
            std::cerr << "exception ShmManager::findName(): " << rNames;
        }
        return value;
    }
    /** removes a named shared memory segment
    * @see bi
    * @param name name of segment to remove
    **/
    static bool removeSegment ( const std::string &name ) {
        return bi::shared_memory_object::remove ( name.c_str() );
    }
    bool removeSegment() {
        return removeSegment ( name_ );
    }

    /** returns namespace
    * @return namespace
    **/
    const std::string &getNamespace() const {
        return namespace_;
    }
    /** Defines a namespace for shared memory varaibles constructed with this handler
    * a "/" will allways present after calling this fnc
    * @param namespace name prefix
    **/
    void setNamespace ( const std::string& ns ) {
        namespace_ = ns;
        boost::trim ( namespace_ );
        boost::trim_left_if ( namespace_, boost::is_any_of ( "/" ) );
        boost::trim_right_if ( namespace_, boost::is_any_of ( "/" ) );
	if(namespace_.empty()) namespace_ = "/";
	else namespace_ = "/" + namespace_ + "/";
    }
    /** adds the prefix to a name
    * @param name
    * @return prefix + name
    **/
    std::string resolve_namespace ( const std::string &name ) {
        if ( namespace_.empty() ) return name;
        return namespace_ + name;
    }
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

