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

#include <iostream>
#include <stdlib.h>
#include "shmfw/header.h"
#include "shmfw/handler_variable.h"

#include <boost/program_options.hpp>

namespace bi = boost::interprocess;
namespace po = boost::program_options;

struct Prarmeters {
    bool clear;
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::string variable_name;
    bool show_hidden;
    bool show_container;
    bool show_lock;
    bool show_timestamp;
    bool show_type;
    bool show_context;
};

Prarmeters readArgs ( int argc, char **argv ) {
    Prarmeters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "clear,c", "clears the shared memory" )
    ( "timestamp,t", "show variable timestamp" )
    ( "context", "show variable context" )
    ( "type", "show variable type" )
    ( "lock,l", "toggel variable lock" )
    ( "hidden", "show hidden variables" )
    ( "container", "toggel variable container type" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm_memory_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size,s", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" )
    ( "variable_name,v", po::value<std::string> ( &params.variable_name ), "shared variable name" );

    po::variables_map vm;
    try {
        po::store ( po::parse_command_line ( argc, argv, desc ), vm );
    } catch ( const std::exception &ex ) {
        std::cout << desc << std::endl;;
        exit ( 1 );
    }
    po::notify ( vm );

    if ( vm.count ( "help" ) )  {
        std::cout << desc << std::endl;
        exit ( 1 );
    }
    params.clear = ( vm.count ( "clear" ) > 0 );
    params.show_context = ( vm.count ( "context" ) > 0 );
    params.show_type = ( vm.count ( "type" ) > 0 );
    params.show_timestamp = ( vm.count ( "timestamp" ) > 0 );
    params.show_lock = !( vm.count ( "lock" ) > 0 );
    params.show_container = !( vm.count ( "container" ) > 0 );
    params.show_hidden = vm.count ( "hidden" ) > 0;

    return params;
}

int main ( int argc, char **argv ) {
    Prarmeters params = readArgs ( argc, argv );
    if ( params.clear ) {
        ShmFw::Handler::removeSegment ( params.shm_memory_name );
        std::cout << "Shared Memory " << params.shm_memory_name << " cleared" << std::endl;
	exit(0);
    }
    std::vector<std::string> varNames;
    std::vector < ShmFw::HandlerObjectPtr > objects;
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    bi::managed_shared_memory::segment_manager *seg_manager = shmHdl->getShm()->get_segment_manager();
    std::cout << std::endl;
    std::cout << "Segment name                    : " << std::setw(10) << params.shm_memory_name << std::endl;
    std::cout << "Wegment size                    : " << std::setw(10) << seg_manager->get_size()        << " bytes = " << seg_manager->get_size()/(1024.*1024.) << " MByte" << std::endl;
    std::cout << "Free memory                     : " << std::setw(10) << seg_manager->get_free_memory() << " bytes = " << seg_manager->get_free_memory()/(1024.*1024.) << " MByte = " << seg_manager->get_free_memory()*100/seg_manager->get_size() <<  " %" << std::endl;
    std::cout << "All memory has been deallocated : " << std::setw(10) << (seg_manager->all_memory_deallocated()?"ture":"false") <<  std::endl;
    std::cout << "Internal structures             : " << std::setw(10) << (seg_manager->check_sanity()?"OK":"errors") <<  std::endl;
    std::cout << "Number of named objects         : " << std::setw(10) << seg_manager->get_num_named_objects() <<  std::endl;
    std::cout << "Number of unique objects        : " << std::setw(10) << seg_manager->get_num_unique_objects() <<  std::endl;
    std::cout << std::endl;
    shmHdl->listNames ( varNames, params.show_hidden);
    for ( unsigned int i = 0; i < varNames.size(); i++ ) {
        ShmFw::Header shmHeader ( shmHdl, varNames[i] );
        std::cout << std::setw ( 3 ) << i << ": " << std::setw ( 30 ) << varNames[i];
        if ( params.show_container ) {
            std::cout << ", " << std::setw ( 12 ) << shmHeader.containerName();
        }
        if ( params.show_context ) {
            std::cout << ( shmHeader.locked() ? ", locked  " : ", unlocked" );
        }
        if ( params.show_context ) {
            if ( ( shmHeader.container() == ShmFw::Header::CONTAINER_VARIABLE ) ||
                    ( shmHeader.container() == ShmFw::Header::CONTAINER_VECTOR ) ) {
                ShmFw::HandlerObjectPtr objects = ShmFw::HandlerObject::open ( varNames[i], shmHdl );
                if ( objects ) {
                    std::cout << " = " << objects->value();
                } else {
                    std::cout << "?";
                }
            } else {
                std::cout << " = unsupported container";
            }
        }
        if ( params.show_type ) {
            std::cout << ", " << std::setw ( 8 )  << shmHeader.type_hash_code() << " : ";
            std::cout << shmHeader.type_name();
        }
        if ( params.show_timestamp ) {
            std::cout << ", " << shmHeader.timestampShm() ;
        }
        std::cout << std::endl;
    }
    exit ( 0 );
}
