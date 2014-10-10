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


#include "shmfw/variable.h"
#include "shmfw/allocator.h"
#include "shmfw/objects/laser_scan.h"
#include <ncurses.h>


#include <boost/program_options.hpp>
#include <boost/thread.hpp>

size_t counter;
size_t activeParameter;

struct Prarmeters {
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::string variable_name;
    bool close;
};

Prarmeters readArgs ( int argc, char **argv ) {
    namespace po = boost::program_options;

    Prarmeters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "variable_name,n", po::value<std::string > ( &params.variable_name ), "shared variable names" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm_memory_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" );

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
    return params;
}



int main ( int argc, char *argv[] ) {

    Prarmeters params = readArgs ( argc, argv );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    ShmFw::Alloc<ShmFw::LaserScan> scan ( params.variable_name, shmHdl );

    initscr();                        // Start curses mode
    raw();                            // Line buffering disabled
    start_color();
    init_pair ( 1, COLOR_WHITE, COLOR_BLACK );
    init_pair ( 1, COLOR_WHITE, COLOR_BLACK );
    init_pair ( 2, COLOR_RED, COLOR_BLACK );
    init_pair ( 3, COLOR_GREEN, COLOR_BLACK );
    init_pair ( 4, COLOR_BLUE, COLOR_BLACK );
    init_pair ( 5, COLOR_RED, COLOR_YELLOW );
    init_pair ( 6, COLOR_WHITE, COLOR_YELLOW );
    keypad ( stdscr, TRUE );          // We get F1, F2 etc..
    noecho();
    int key = ' ';
    int rows,cols;
    getmaxyx ( stdscr,rows,cols );
    attron ( COLOR_PAIR ( 3 ) );
    mvprintw ( 0, cols/2-5, "View LaserScan: %s",  params.variable_name.c_str() );
    attron ( COLOR_PAIR ( 1 ) );
    mvprintw ( rows-1, cols-20, "press 'Esc' for quit" );
    attron ( COLOR_PAIR ( 1 ) );
    int firstRow = 3;
    while ( params.close == false ) {           // Don't echo() while we do getch
        std::stringstream message;
        std::stringstream message_error;
        getmaxyx ( stdscr,rows,cols );
	int row = firstRow;
        mvprintw ( row++, 0, "frame: %s", scan().frame.c_str());
        mvprintw ( row++, 0, "range: %fm - %fm", scan().range_min, scan().range_max);
        mvprintw ( row++, 0, "angle: %frad - %frad", scan().angle_min, scan().angle_max);
        mvprintw ( row++, 0, "angle_increment: %f", scan().angle_increment);
        mvprintw ( row++, 0, "time_increment: %f", scan().time_increment);
        mvprintw ( row++, 0, "scan_time: %4.2f  --> %4.2fHz", scan().scan_time, 1000/scan().scan_time);
        mvprintw ( row++, 0, "size:  %i", scan().ranges.size());
	int step_col = 5;
	int step_size = scan().ranges.size() / (cols / step_col);
	for(size_t i = 0, col = 0; i < scan().ranges.size(); i+=step_size, col += step_col) {
	    mvprintw ( row,   col, "%d", i);
	    mvprintw ( row+1, col, "%4.1f", scan().ranges[i]);
	}   
        timeout ( 100 );
        key = getch();
        switch ( key ) {
        case 27: //Esc
            params.close = true;
            break;
        }
    }
    endwin();
    exit ( 0 );
}



