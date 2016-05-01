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
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string/split.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#include <boost/iostreams/stream.hpp>

#include "shmfw/com/udp.h"
#include "shmfw/base_types.h"
#include "shmfw/variable.h"
#include "shmfw/vector.h"

struct Prarmeters {
    std::string host;
    std::string prefix;
    unsigned short port;
    bool receiver;
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::vector<std::string> variable_names;
    std::string configFile;
    int message_format;
    unsigned int skip_count;
    unsigned int info_string_intervall;
    bool print_messages;
};

Prarmeters params;
std::shared_ptr< ShmFw::Handler > pShmHdl;
std::vector< std::shared_ptr< ShmFw::Vector<double> > > shmData;
std::shared_ptr< ShmFw::UDP > pUDP;
unsigned int message_count;


namespace ShmFw {
template void Vector<double>::serialize<boost::archive::xml_iarchive>(boost::archive::xml_iarchive&, unsigned int);
template void Vector<double>::serialize<boost::archive::xml_oarchive>(boost::archive::xml_oarchive&, unsigned int);  // explicit instantiation
};


int program_options ( int argc, char* argv[], Prarmeters &params ) {
    boost::program_options::options_description desc ( "Allowed Parameters" );
    desc.add_options() //
    ( "help", "get this help message" ) //
    ( "cfg", boost::program_options::value<std::string> ( &params.configFile ), "Config file" ) //
    ( "receiver,r", "if set the program acts as receiver: default is transmitter" ) //
    ( "host,h", boost::program_options::value<std::string> ( &params.host )->default_value ( "127.0.0.1" ), "Host" ) //
    ( "port,p", boost::program_options::value<unsigned short> ( &params.port )->default_value ( 25000 ), "Port" )   //
    ( "prefix,x", boost::program_options::value<std::string> ( &params.prefix )->default_value ( "udp" ), "prefix for message" ) //
    ( "skip_count", boost::program_options::value<unsigned int> ( &params.skip_count )->default_value ( 1 ), "skip count nr of changes/messges skipped" ) //
    ( "info_string_intervall,i", boost::program_options::value<unsigned int> ( &params.info_string_intervall )->default_value ( 100 ), "Info string intervall zero means no info string" )   //
    ( "print_messages", "prints the hole message to cout" )    //
    ( "message_format,f", boost::program_options::value<int> ( &params.message_format )->default_value ( 0 ), "Message Foramt 1=xml 2=binar" )   //
    ( "shm_memory_name,m", boost::program_options::value<std::string> ( &params.shm_memory_name )->default_value ( DEFAULT_SEGMENT_NAME ), "shared memory segment name" )
    ( "shm_memory_size,s", boost::program_options::value<unsigned int> ( &params.shm_memory_size )->default_value ( DEFAULT_SEGMENT_SIZE ), "shared memory segment size" )
    ( "variable_names,v", boost::program_options::value<std::vector<std::string> > ( &params.variable_names )->default_value ( std::vector<std::string>(),"data" )->multitoken(), "shared variable" );


    std::cout << std::endl;

    boost::program_options::variables_map vm;
    try {
        boost::program_options::store ( boost::program_options::parse_command_line ( argc, argv, desc ), vm );
    } catch ( const std::exception& ex ) {
        std::cout << desc << "\n";
        return 1;
    }
    boost::program_options::notify ( vm );

    if ( vm.count ( "help" ) ) {
        std::cout << desc << "\n";
        exit ( 0 );
    }

    if ( ( params.message_format < 0 ) || ( params.message_format > 1 ) ) {
        std::cout << "--> wrong message foramt \n\n";
        std::cout << desc << "\n";
        exit ( 0 );
    }

    params.receiver = ( vm.count ( "receiver" ) > 0 );
    params.print_messages = ( vm.count ( "print_messages" ) > 0 );

    if ( !params.configFile.empty() ) {
        std::ifstream file ( params.configFile.c_str(), std::ifstream::in );
        if ( file.is_open() ) {
            try {
                boost::program_options::store ( boost::program_options::parse_config_file ( file, desc ), vm );
                boost::program_options::notify ( vm );
            } catch ( const std::exception& ex ) {
                std::cout << "Error reading config file: " << ex.what() << std::endl;
                return 1;
            }
        } else {
            std::cout << "Error opening config file " << params.configFile << std::endl;
            return 1;
        }
    }
    return 0;
}


void print_message ( std::string &msg ) {
    if ( ( params.info_string_intervall != 0 ) && ( message_count%params.info_string_intervall == 0 ) ) {
        std::cout << std::setw ( 4 ) << message_count  << "msgs with " << msg.size() << " bytes!" << std::endl;
        if ( params.print_messages ) {
            std::cout << msg << std::endl;
        }
    }
}

void incomming ( std::shared_ptr<std::string> &msgPtr ) {
    message_count++;
    boost::iostreams::basic_array_source<char> device ( msgPtr->data(), msgPtr->size() );
    boost::iostreams::stream<boost::iostreams::basic_array_source<char> > s ( device );

    print_message ( *msgPtr );
    std::shared_ptr< boost::archive::xml_iarchive> xmlPtr;
    std::shared_ptr< boost::archive::binary_iarchive> binaryPtr;
    if ( params.message_format == 0 )  xmlPtr = std::shared_ptr< boost::archive::xml_iarchive> ( new boost::archive::xml_iarchive ( s ) );
    else if ( params.message_format == 1 )  binaryPtr = std::shared_ptr< boost::archive::binary_iarchive> ( new boost::archive::binary_iarchive ( s ) );

    for ( unsigned int i = 0; i < shmData.size(); i++ ) {
        if ( params.message_format == 0 ) ( *xmlPtr ) >> boost::serialization::make_nvp ( shmData[i]->name().c_str(), *shmData[i] );
        else if ( params.message_format == 1 ) ( *binaryPtr ) >> boost::serialization::make_nvp ( shmData[i]->name().c_str(), *shmData[i] );
    }
}

void doSomeThing ( ShmFw::OsziChannels &data ) {
    std::cout << data << std::endl;
}

bool loop_program = true;

void terminate ( int param ) {
    std::cout << "Program was trying to abort or terminate." << std::endl;
    loop_program = false;
}

int main ( int argc, char *argv[] ) {

    signal ( SIGABRT,	terminate );
    signal ( SIGTERM,	terminate );
    using boost::asio::ip::udp;
    program_options ( argc, argv, params );

    pShmHdl = std::shared_ptr< ShmFw::Handler > ( new ShmFw::Handler ( params.shm_memory_name, params.shm_memory_size ) );
    for ( unsigned int i = 0; i < params.variable_names.size(); i++ ) {
        shmData.push_back ( std::shared_ptr< ShmFw::Vector<double> > ( new ShmFw::Vector<double> ( params.variable_names[i], *pShmHdl ) ) );
    }
    pUDP = std::shared_ptr< ShmFw::UDP > ( new ShmFw::UDP() );
    message_count = 0;

    try {
        if ( params.receiver ) {
            pUDP->initReceiver ( params.port );
            pUDP->setCallback ( &incomming );
            pUDP->runThread();
            for ( unsigned int waitCounter = 0; loop_program; waitCounter++ ) {
                while ( shmData[0]->timed_wait ( 1000 ) == false ) {
                    std::cout << std::setw ( 4 ) << waitCounter << ": waited 1000 ms" << std::endl;
                }
            }
            pUDP->stopThread();

        } else {
            pUDP->initTransmitter ( params.host, params.port );
            for ( unsigned int waitCounter = 0; loop_program; waitCounter++ ) {
                while ( shmData[0]->timed_wait ( 1000 ) == false ) {
                    std::cout << std::setw ( 4 ) << waitCounter << ": waited 1000 ms" << std::endl;
                }
		if((waitCounter % (params.skip_count+1)) != 0) continue;
                std::string msg;
                boost::iostreams::back_insert_device<std::string> inserter ( msg );
                boost::iostreams::stream<boost::iostreams::back_insert_device<std::string> > os ( inserter );
				
                std::shared_ptr< boost::archive::xml_oarchive> xmlPtr;
                std::shared_ptr< boost::archive::binary_oarchive> binaryPtr;
                if ( params.message_format == 0 )  xmlPtr = std::shared_ptr< boost::archive::xml_oarchive> ( new boost::archive::xml_oarchive ( os ) );
                else if ( params.message_format == 1 )  binaryPtr = std::shared_ptr< boost::archive::binary_oarchive> ( new boost::archive::binary_oarchive ( os ) );

                for ( unsigned int i = 0; i < shmData.size(); i++ ) {
                    if ( params.message_format == 0 ) ( *xmlPtr ) << boost::serialization::make_nvp ( shmData[i]->name().c_str(), *shmData[i] );
                    else if ( params.message_format == 1 ) ( *binaryPtr ) << boost::serialization::make_nvp ( shmData[i]->name().c_str(), *shmData[i] );
                }
                
                os.flush();
                pUDP->send ( msg.c_str(), msg.size() );
                message_count++;
                print_message ( msg );
            }
        }

    } catch ( std::exception& e ) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}

