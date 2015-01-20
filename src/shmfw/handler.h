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
#include <iostream>
#include <boost/interprocess/interprocess_fwd.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

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
typedef boost::shared_ptr<bi::managed_shared_memory> ShmPtr;

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
     * @param ns namespace
     **/
    Handler ( const std::string &name, size_t size = DEFAULT_SEGMENT_SIZE(), const std::string &ns = "");
    /**
     * Construtor creates a shm
     * @param p shared memory parameter
     **/
    Handler (const ParameterPtr &p );
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
    static HandlerPtr create ( const std::string &name, size_t size = DEFAULT_SEGMENT_SIZE(), const std::string &ns = "" );
     /**
     * rerates a new handler
     * @param p shared memory parameter
     **/
    static HandlerPtr create (const  ParameterPtr &p );
    /**
    * @return managed memory
    **/
    ShmPtr getShm();
    /** creates a named shared memory segment with the current name and size
    **/
    void createSegment();
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
    /** adds the prefix to a name
    * @param name
    * @return prefix + name
    **/
    std::string resolve_namespace ( const std::string &_name ) ;
private:
    Handler ( const Handler & ) {};
    bool valid_;
    ParameterPtr param_;
    ShmPtr pShm_;
};


};


#endif //SHARED_MEM_HANDLER_H

