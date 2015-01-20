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

#include "shmfw.h"
#include <fstream>      // std::ifstream
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>

using namespace ShmFw;

ParameterPtr Parameter::create() {
    return ParameterPtr ( new Parameter );
}
ParameterPtr Parameter::create ( const std::string& name, unsigned int size, const std::string& ns ) {
    return ParameterPtr ( new Parameter ( name, size, ns ) );
}
Parameter::Parameter()
    : segment_name ( DEFAULT_SEGMENT_NAME() )
    , segment_size ( DEFAULT_SEGMENT_SIZE() ) //16MB;
    , segment_ns ( "/" ) {
};
Parameter::Parameter ( const std::string& name, unsigned int size, const std::string& ns )
    : segment_name ( name )
    , segment_size ( size ) //16MB;
    , segment_ns ( ns) {
      fix_namespace_syntax();
};
void Parameter::fix_namespace_syntax(){
    boost::trim ( this->segment_ns );
    boost::trim_left_if ( this->segment_ns, boost::is_any_of ( "/" ) );
    boost::trim_right_if ( this->segment_ns, boost::is_any_of ( "/" ) );
    if ( this->segment_ns.empty() ) this->segment_ns = "/";
    else this->segment_ns = "/" + this->segment_ns + "/";
}

std::string Parameter::resolve_namespace ( const std::string &_name ) {
    if ( this->segment_ns.empty() ) return _name;
    std::string n = _name;
    boost::trim_left_if ( n, boost::is_any_of ( "/" ) );
    boost::trim_right_if ( n, boost::is_any_of ( "/" ) );
    return this->segment_ns + n;
}

void Parameter::parse ( int argc, char **argv, const std::string &group, std::string &filename ){
    namespace po = boost::program_options;
    po::options_description desc ( "Allowed Arguments" );
    desc.add_options()
    ( "shm.memory_name,m", po::value<std::string> ( &this->segment_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm.memory_size,s", po::value<size_t> ( &this->segment_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" )
    ( "shm.namespace", po::value<std::string> ( &this->segment_name )->default_value ( "r1" ), "shm namespace (robot name) " );

    // Load setting file.
    po::variables_map vm;
    try {
        po::store ( po::parse_command_line ( argc, argv, desc ), vm );
        po::notify ( vm );
        if ( !filename.empty() )  {
            std::ifstream ifs ( filename , std::ifstream::in );
            if ( !ifs ) {
                std::cout << "can not open config file: " << filename << std::endl;
                return;
            }
            po::store ( po::parse_config_file ( ifs , desc ), vm );
            po::notify ( vm );
            ifs.close();
        }
    } catch ( const std::exception &ex ) {
        std::cout << desc << std::endl;;
        return;
    }
    
}

boost::posix_time::ptime ShmFw::now() {
    return boost::posix_time::microsec_clock::local_time();
}

std::string ShmFw::DEFAULT_SEGMENT_NAME() {
    return "ShmFw";
};
size_t ShmFw::DEFAULT_SEGMENT_SIZE() {
    return 16*1024*1024; //16MB;
};
