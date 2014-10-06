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
#include <signal.h>


#include "shmfw/variable.h"
#include "shmfw/vector.h"
#include "shmfw/log.h"
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <ncurses.h>

bool loop_program = true;

struct Prarmeters {
    bool clear;
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::string variable_name;
    std::string log_file;
    int process_cout_level;
    int source;
    int file_level;
    int cout_level;
    bool timeout_msg;
    bool pull;
    bool buffer;
    bool timestamp;
    int timeout;
};

Prarmeters readArgs ( int argc, char **argv ) {
    namespace po = boost::program_options;

    Prarmeters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "clear", "clears the shared memory" )
    ( "msg_off,o", "switches the timeout message off" )
    ( "buffer", "shows the buffer id" )
    ( "timestamp_off", "switches the timestamp off" )
    ( "pull,p", "pulls for log messages (no trigger needed)" )
    ( "timeout,t", po::value<int> ( &params.timeout )->default_value ( 2000 ), "timeout for timeout message in ms" )
    ( "source,s", po::value<int> ( &params.source )->default_value ( -1 ), "source filter (-1 off)" )
    ( "process_cout_level", po::value<int> ( &params.process_cout_level )->default_value ( ShmFw::Message::NA ), "level of message to be printed by the meassage creating process" )
    ( "cout_level", po::value<int> ( &params.cout_level )->default_value ( ShmFw::Message::NA ), "level of message to be printed by this process" )
    ( "file_level", po::value<int> ( &params.file_level )->default_value ( ShmFw::Message::NA ), "level of message to be stored" )
    ( "file,f", po::value<std::string> ( &params.log_file )->default_value ( "/tmp/log" ), "log file prefix, if empty it will print of cout" )
    ( "shm_log,l", po::value<std::string> ( &params.variable_name )->default_value ( "log" ), "shared variable name of the logger" )
    ( "shm_memory_name", po::value<std::string> ( &params.shm_memory_name )->default_value ( LOG_SEGMENT_NAME ), "shared memory segment name" )
    ( "shm_memory_size", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( LOG_SEGMENT_SIZE ), "shared memory segment size" );

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
    params.timeout_msg = ! ( vm.count ( "msg_off" ) > 0 );
    params.pull = ( vm.count ( "pull" ) > 0 );
    params.buffer = ( vm.count ( "buffer" ) > 0 );
    params.timestamp = !( vm.count ( "timestamp_off" ) > 0 );

    return params;
}

void printMsgs ( std::vector<ShmFw::Message> &msgs, std::ofstream &file, const Prarmeters &params ) {

    for ( unsigned int i = 0; i < msgs.size(); i++ ) {
	ShmFw::Message &msg = msgs[i];
        if ( !params.log_file.empty() ) {
            file << std::setw ( 4 ) << i << ": " << msg << std::endl;
        }
	//source log level/type filter
        if ( msg.getType() >= params.cout_level ) {
            if ( params.buffer )  std::cout << std::setw ( 4 ) << i << ": ";
	    if ( params.timestamp )  std::cout << boost::posix_time::to_simple_string ( msg.getTime() ).substr(12,26) << ": ";
	    if ( msg.getType() != ShmFw::Message::NA ) std::cout << ": " <<  std::setw ( 8 ) << msg.typeStr() <<  ": ";	    
	    std::cout << msg.getMsg();
	    std::cout << std::endl;
        }
    }
}

void dequeLog ( ShmFw::Log &log, const Prarmeters &params ) {
    std::vector<ShmFw::Message> msgs;
    std::ofstream file;
    boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
    std::string datedFileName = params.log_file + std::string ( "_" ) + boost::posix_time::to_iso_string ( now ) + std::string ( ".txt" );
    if ( !params.log_file.empty() ) {
        file.open ( datedFileName.c_str(), std::ios::out | std::ios::binary );
    }
    while ( loop_program ) {
        bool wait_ones = true;
        while ( ( log.timed_wait ( params.timeout ) == false ) && loop_program && wait_ones ) {
            if ( params.timeout_msg ) std::cout << "waited " << params.timeout << " ms" << std::endl;
            if ( params.pull ) wait_ones = false;
        }
        msgs.clear();
        log.lock();
        for ( ShmFw::Log::Iterator it = log().begin(); it != log().end(); it++ ) {
	    //source filter
            if ( (params.source < 0) || (params.source == ( *it ).getSource())) {
                msgs.push_back ( *it );
                log.pop_front();
	    }
        }
        log.unlock();
        printMsgs ( msgs, file, params );
    }
    if ( !params.log_file.empty() ) file.close();
}


void terminate ( int param ) {
    std::cout << "Closing program!" << std::endl;
    loop_program = false;
}

int main ( int argc, char *argv[] ) {

    signal ( SIGINT, terminate );
    signal ( SIGKILL, terminate );

    Prarmeters params = readArgs ( argc, argv );
    if ( params.clear ) {
        ShmFw::Handler::removeSegment ( params.shm_memory_name );
        std::cout << "Shared Memory " << params.shm_memory_name << " cleared" << std::endl;
        exit ( 1 );
    }
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    ShmFw::Log log ( shmHdl, params.variable_name );
    log.process_cout_level ( params.process_cout_level );
    log.unlock();
    dequeLog ( log, params );

    log.unlock();

    exit ( 0 );


}



