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


#include <mgl2/qt.h>
//#include <mgl2/fltk.h>
//#include <mgl2/window.h>
#include <shmfw/objects/dynamic_grid_map.h>
#include <opencv2/highgui/highgui.hpp>
#include <shmfw/allocator.h>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

mglQT *gr=NULL;

struct Prarmeters {
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::string variable_name;
    ShmFw::HandlerPtr shmHdl;
    bool loop;
    std::string title;
    std::string arguments;
    std::string plot_type;
    unsigned int update_time;
};
Prarmeters params;


void terminate ( int s ) {
    printf ( "Caught signal %d\n",s );
    fflush ( stdout );
    params.loop = false;
}

void readArgs ( int argc, char **argv, Prarmeters &params ) {
    namespace po = boost::program_options;

    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "clear,c", "clears the shared memory" )
    ( "loop,l", "loops writing data" )
    ( "title,t", po::value<std::string> ( &params.title )->default_value ( "2D" ), "plot title" )
    ( "plot_type,p", po::value<std::string> ( &params.plot_type )->default_value ( "mesh" ), "plot type: mesh, fall, belt, boxs, tile, dens  ... " )
    ( "arguments,a", po::value<std::string> ( &params.arguments )->default_value ( "" ), "plot arguments" )
    ( "update_time,u", po::value<unsigned int> ( &params.update_time )->default_value ( 5000 ), "update time in ms" )
    ( "variable_name,v", po::value<std::string> ( &params.variable_name )->default_value ( "Grid" ), "shared variable name" )
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
    params.loop = ( vm.count ( "loop" ) > 0 );
}

void prepare_grid ( mglData* data ) {
}

int capture_data () {
    params.shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    ShmFw::Alloc<ShmFw::DynamicGridMap64FShm> dynamic_grid ( params.variable_name, params.shmHdl );
    size_t col=0, row=0, columns=0, rows=0;
    unsigned int loop_count = 0;
    int update_count = 0;
    int timeout_count = 0;
    while ( gr == NULL ) {
        sleep ( 1 );
    }
    do {
        mglData a;
        if ( dynamic_grid.timed_wait ( params.update_time ) ) {
            update_count++;
            timeout_count = 0;
            std::cout << "update_count:" << update_count << std::endl;
        } else {
            timeout_count++;
            std::cout << "timeout_count:" << timeout_count << std::endl;
        }
        columns = dynamic_grid->getSizeX();
        rows = dynamic_grid->getSizeY();
        mgl_data_create ( &a, rows, columns, 1 );

        for ( col = 0; col < columns; col++ )  {
            for ( row=0; row < rows; row++ ) {
                double v = dynamic_grid->cellByIndex_nocheck ( col,row );
                mgl_data_set_value ( &a, v, row, col, 0 );
            }
        }
        gr->Clf();      // make new drawing
        usleep ( 10000 );
        gr->Title ( params.title.c_str() );
        gr->Rotate ( 50,60 );
        gr->Box();
        if ( boost::iequals ( "mesh", params.plot_type ) ) {
            if ( params.arguments.empty() ) gr->Mesh ( a );
            else gr->Mesh ( a, params.arguments.c_str() );
        } else if ( boost::iequals ( "fall", params.plot_type ) ) {
            if ( params.arguments.empty() ) gr->Fall ( a );
            else gr->Fall ( a, params.arguments.c_str() );
        } else if ( boost::iequals ( "belt", params.plot_type ) ) {
            if ( params.arguments.empty() ) gr->Belt ( a );
            else gr->Belt ( a, params.arguments.c_str() );
        } else if ( boost::iequals ( "boxs", params.plot_type ) ) {
            if ( params.arguments.empty() ) gr->Boxs ( a );
            else gr->Boxs ( a, params.arguments.c_str() );
        } else if ( boost::iequals ( "tile", params.plot_type ) ) {
            if ( params.arguments.empty() ) gr->Tile ( a );
            else gr->Tile ( a, params.arguments.c_str() );
        } else if ( boost::iequals ( "dens", params.plot_type ) ) {
            if ( params.arguments.empty() ) gr->Dens ( a );
            else gr->Dens ( a, params.arguments.c_str() );
        } else if ( boost::iequals ( "cont", params.plot_type ) ) {
            if ( params.arguments.empty() ) gr->Cont ( a );
            else gr->Cont ( a, params.arguments.c_str() );
        }
        gr->SetRanges ( dynamic_grid->getXMin(),dynamic_grid->getXMax(),dynamic_grid->getYMin(),dynamic_grid->getYMax() );
        gr->Axis();
        gr->Grid();
        gr->Update();
        loop_count ++;
    } while ( params.loop );
    exit ( 0 );
}

int main ( int argc, char **argv ) {
    readArgs ( argc, argv, params );
    signal ( SIGABRT,	terminate );
    signal ( SIGTERM,	terminate );

    gr = new mglQT;
    sleep ( 1 );
    boost::thread data_update ( capture_data );
    gr->Run();
    return 0;
}

