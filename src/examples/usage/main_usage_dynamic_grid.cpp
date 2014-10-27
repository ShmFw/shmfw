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

#include <shmfw/objects/dynamic_grid.h>
#include <shmfw/allocator.h>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/chrono/thread_clock.hpp>

struct Prarmeters {
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::string variable_name;
    bool loop;
};
Prarmeters params;

void readArgs ( int argc, char **argv, Prarmeters &params ) {
    namespace po = boost::program_options;

    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "clear,c", "clears the shared memory" )
    ( "loop,l", "loops writing data" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm_memory_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size,s", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" );

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
    params.loop = ! ( vm.count ( "loop" ) > 0 );
}

void terminate ( int s ) {
    printf ( "Caught signal %d\n",s );
    fflush ( stdout );
    params.loop = false;
}
int main ( int argc, char *argv[] ) {


    using namespace boost::chrono;
    typedef boost::chrono::thread_clock tclock;
    thread_clock::time_point thStart, thStop;

    readArgs ( argc, argv, params );
    signal ( SIGABRT,	terminate );
    signal ( SIGTERM,	terminate );

    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    srand ( time ( NULL ) );

    ShmFw::DynamicGrid64FHeap gridHeap1;
    ShmFw::DynamicGrid64FHeap gridHeap2;
    ShmFw::Alloc<ShmFw::DynamicGrid64FShm> gridShm ( "Grid", shmHdl);

    gridHeap1.setSize ( 0, 50, 0, 40, 1, 0 );
    std::cout << gridHeap1 << std::endl;
    int count = 0;
    do {
        double d = sin ( count / 100. );
        double x, y, m = gridHeap1.getSizeX(), n = gridHeap1.getSizeY();
        for ( int i=0; i<m; i++ ) {
            for ( int j=0; j<n; j++ ) {
                x = i/ ( n-1. );
                y = j/ ( m-1. );
                double v = d*sin ( 2*M_PI*x ) *sin ( 3*M_PI*y ) + ( 1.0-d ) *cos ( 3*M_PI*x*y );
                gridHeap1.setCellByIndex ( i, j, v );
            }
        }
        std::cout << std::endl;
        thStart = tclock::now();
        gridHeap1.copyTo ( gridHeap2 );
        thStop = tclock::now();
        std::cout << "heap->heap: copyTo using_memcpy  : " << duration_cast<nanoseconds> ( thStop - thStart ).count() << " nanoseconds\n";
        thStart = tclock::now();
        gridHeap1.copyTo ( gridHeap2 );
        thStop = tclock::now();
        std::cout << "heap->heap: copyTo using_forloop : " << duration_cast<nanoseconds> ( thStop - thStart ).count() << " nanoseconds\n";
        thStart = tclock::now();
        gridHeap1.copyTo ( *gridShm );
        thStop = tclock::now();
        std::cout << "heap->shm: copyTo using_memcpy   : " << duration_cast<nanoseconds> ( thStop - thStart ).count() << " nanoseconds\n";
        thStart = tclock::now();
        gridHeap1.copyTo ( *gridShm );
        thStop = tclock::now();
        std::cout << "heap->shm: copyTo using_forloop  : " << duration_cast<nanoseconds> ( thStop - thStart ).count() << " nanoseconds\n";
        gridShm.itHasChanged();
        count++;
        usleep ( 100000 );
    } while ( params.loop );

    exit ( 0 );

}
