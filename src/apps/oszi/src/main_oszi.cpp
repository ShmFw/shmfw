#include <iostream>
#include <stdlib.h>
#include <signal.h>
#include <stdarg.h>

#include "shmfw/oszi.h"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

struct Prarmeters {
    bool demo;
    bool capture_images;
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::string variable_name;
    std::string config_oszi;
};

Prarmeters readArgs ( int argc, char **argv ) {
    namespace po = boost::program_options;

    Prarmeters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "demo", "starts a thred to gernerate demo data" )
	( "capture_images", "records images into a temp folder" )
    ( "config_oszi,o", po::value<std::string> ( &params.config_oszi )->default_value ( "oszi.cfg" ), "Oszi config" )
    ( "variable_name,v", po::value<std::string> ( &params.variable_name )->default_value ( "oszi" ), "shared variable name" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm_memory_name )->default_value ( DEFAULT_SEGMENT_NAME ), "shared memory segment name" )
    ( "shm_memory_size,s", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( DEFAULT_SEGMENT_SIZE ), "shared memory segment size" );

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
    params.demo = ( vm.count ( "demo" ) > 0 );
    params.capture_images = ( vm.count ( "capture_images" ) > 0 );

    return params;
}

bool loop_program = true;
void terminate ( int param ) {
    std::cout << "Closing program!" << std::endl;
    loop_program = false;
}


void demo_data_generator ( std::string *name, ShmFw::HandlerPtr shmHdl ) {
    ShmFw::Vector<double> channelsSin ( *name, shmHdl );
    ShmFw::Vector<double> channelsCos ( "cos", shmHdl );
    ShmFw::Vector<double> channelsRect ( "rect", shmHdl );
    double sig = 0;
    channelsSin.resize ( 2 );
    channelsCos.resize ( 1 );
    channelsRect.resize ( 1 );
    for ( unsigned int i = 0; loop_program; i++ ) {
        sig += 0.005;
        if ( sig > 2 * M_PI ) sig = 0;
        channelsSin.lock();
        channelsSin.at ( 0 ) = sin ( sig );
        channelsSin.at ( 1 ) = sig;
        channelsSin.unlock();
        channelsSin.itHasChanged();
        channelsCos.lock();
        channelsCos.at ( 0 ) = cos ( sig );
        channelsCos.unlock();
        channelsCos.itHasChanged();
        channelsRect.lock();
        channelsRect.at ( 0 ) = ( sig>M_PI?-0.5:0.5 );
        channelsRect.unlock();
        channelsRect.itHasChanged();
        usleep ( 1000 );
    }
};


int main ( int argc, char *argv[] ) {

    signal ( SIGINT, terminate );
    signal ( SIGKILL, terminate );
    Prarmeters params = readArgs ( argc, argv );
    
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    ShmFw::Oszi oszi ( params.variable_name, shmHdl );

    if ( params.demo ) {
        boost::thread t1 ( demo_data_generator, &params.variable_name, shmHdl );
    }

    boost::filesystem::path p ( params.config_oszi );
    if ( boost::filesystem::exists ( p ) ) {
        oszi.read ( params.config_oszi );
    } else {
        oszi.write ( params.config_oszi );
    }

    oszi.captureImages(params.capture_images);
    oszi.start();
	
    while ( loop_program && !oszi.close()) {
        oszi.show ( 100 );
    }
    exit ( 0 );
}
