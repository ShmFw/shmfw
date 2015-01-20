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

#include "handler.h"
#include <boost/algorithm/string.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

using namespace ShmFw;

Handler::Handler()
    : valid_ ( false ), param_ ( Parameter::create() ) {
}

Handler::Handler (const ParameterPtr& p )
    : valid_ ( false ), param_ ( p ) {
    createSegment();
}

Handler::Handler ( const std::string &name, size_t size, const std::string &ns )
    : valid_ ( false ), param_ ( Parameter::create ( name , size, ns ) ) {
    createSegment();
}

HandlerPtr Handler::create() {
    return HandlerPtr ( new Handler );
}

HandlerPtr Handler::create ( const std::string &name, size_t size, const std::string &ns ) {
    return HandlerPtr ( new Handler ( name, size, ns ) );
}
HandlerPtr Handler::create (const ParameterPtr& p ) {
    return HandlerPtr ( new Handler ( p ) );
}

ShmPtr Handler::getShm() {
    return pShm_;
}
void Handler::createSegment() {
    param_->fix_namespace_syntax();
    try {
        /// look if it allready exists
        pShm_ = ShmPtr ( new bi::managed_shared_memory ( bi::open_only, param_->segment_name.c_str() ) );
        if ( pShm_->get_size() != param_->segment_size ) {
            if ( pShm_->get_size() > param_->segment_size ) {
                std::cerr << "Shared memory segment: " << param_->segment_name << " exists but is bigger I will use this one";
            } else {
                std::cerr << "Shared memory segment: " << param_->segment_name << " exists with a samller size that can be a problem: " << pShm_->get_size() << " != " << param_->segment_size << std::endl;
            }
        }
    } catch ( ... ) {
        /// it did not exist so I will create it
        try {
            pShm_ = ShmPtr ( new bi::managed_shared_memory ( bi::create_only, param_->segment_name.c_str(), param_->segment_size ) );
            /// Clear the memory
            //std::memset(pShm->get_address(), 0, pShm->get_size());
        } catch ( bi::interprocess_exception &ex ) {
            std::cerr << "Problem on crationg the shared memory!" << std::endl;
            std::cout << ex.what() << std::endl;
            exit ( 1 );
        }
    }
    valid_ = true;
}

const std::string &Handler::getName() const {
    return param_->segment_name;
};
unsigned int Handler::getSize() const {
    return param_->segment_size;
};
bool Handler::isValid() {
    valid_ = false;
    try {
        bi::managed_shared_memory shm ( bi::open_only, param_->segment_name.c_str() );
        valid_ = true;
    } catch ( bi::interprocess_exception &ex ) {
        std::cerr << "shared memory does not exist!: " << ex.what() << std::endl;
    }
    return valid_;
}
void Handler::listNames ( std::vector< std::string> &rNames, bool list_hidden ) {
    try {
        typedef bi::managed_shared_memory::const_named_iterator const_named_it;
        const_named_it named_beg = pShm_->named_begin();
        const_named_it named_end = pShm_->named_end();
        for ( ; named_beg != named_end; ++named_beg ) {
            const bi::managed_shared_memory::char_type *name = named_beg->name();
            std::string entryName ( name );
            if ( list_hidden || ( entryName.find_first_of ( "." ) != 0 ) ) {
                rNames.push_back ( name );
            }
        }
    }    catch ( ... ) {
        //AK_LOG_ERROR << "exception ShmManager::listNames()";
        std::cerr << "exception ShmManager::listNames()";
    }
}

void *Handler::findName ( const std::string &rNames, const std::string &prefix ) {
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


bool Handler::removeSegment ( const std::string &name ) {
    return bi::shared_memory_object::remove ( name.c_str() );
}
bool Handler::removeSegment() {
    return removeSegment ( param_->segment_name );
}

const std::string &Handler::getNamespace() const {
    return param_->segment_ns;
}

std::string Handler::resolve_namespace ( const std::string &_name ) {
    return param_->resolve_namespace(_name);
}
