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
    bool show_context;
};

Prarmeters readArgs(int argc, char **argv)
{
    Prarmeters params;
    po::options_description desc("Allowed Parameters");
    desc.add_options()
    ("help", "get this help message")
    ("clear,c", "clears the shared memory")
    ("context", "show variable context")
    ("shm_memory_name,m", po::value<std::string>(&params.shm_memory_name)->default_value(ShmFw::DEFAULT_SEGMENT_NAME()), "shared memory segment name")
    ("shm_memory_size,s", po::value<unsigned int>(&params.shm_memory_size)->default_value(ShmFw::DEFAULT_SEGMENT_SIZE()), "shared memory segment size")
    ("variable_name,v", po::value<std::string>(&params.variable_name), "shared variable name");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch(const std::exception &ex) {
        std::cout << desc << std::endl;;
        exit(1);
    }
    po::notify(vm);

    if(vm.count("help"))  {
        std::cout << desc << std::endl;
        exit(1);
    }
    params.clear = (vm.count("clear") > 0);
    params.show_context = (vm.count("context") > 0);

    return params;
}

int main(int argc, char **argv)
{
    Prarmeters params = readArgs(argc, argv);
    if(params.clear) {
        ShmFw::Handler::removeSegment(params.shm_memory_name);
        std::cout << "Shared Memory " << params.shm_memory_name << " cleared" << std::endl;
    }
    std::vector<std::string> varNames;
    std::vector < ShmFw::HandlerObjectPtr > objects;
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create(params.shm_memory_name, params.shm_memory_size);
    shmHdl->listNames(varNames);
    for(unsigned int i = 0; i < varNames.size(); i++) {
        ShmFw::Header shmHeader(shmHdl, varNames[i]);
        std::cout << std::setw ( 3 ) << i << ": " << std::setw ( 30 ) << varNames[i] << " = ";
        std::cout << std::setw ( 12 ) << shmHeader.containerName();
        std::cout << ( shmHeader.locked() ? ", locked  " : ", unlocked" );
	std::cout << " : ";
	if(params.show_context){
	  if((shmHeader.container() == ShmFw::Header::CONTAINER_VARIABLE) || 
	    (shmHeader.container() == ShmFw::Header::CONTAINER_VECTOR) ){
	      ShmFw::HandlerObjectPtr objects = ShmFw::HandlerObject::open(varNames[i], shmHdl);
	      if(objects){
		std::cout << objects->value();
	      } else {
		std::cout << "?";
	      }
	  } else {
	      std::cout << "-";
	  }
	} else {
	  std::cout << shmHeader.timestampShm() ;
	}
        std::cout << std::endl;
    }
    exit(0);
}
