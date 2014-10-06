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
#include "shmfw/objects/twist.h"
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
    ( "variable_name,n", po::value<std::string > ( &params.variable_name )->default_value ( "cmd_vel" ), "shared variable names" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm_memory_name )->default_value ( DEFAULT_SEGMENT_NAME ), "shared memory segment name" )
    ( "shm_memory_size", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( DEFAULT_SEGMENT_SIZE ), "shared memory segment size" );

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
    ShmFw::Var<ShmFw::Twist> twist ( params.variable_name, shmHdl );

    
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
    mvprintw ( 0, cols/2-5, "View twist: %s",  params.variable_name.c_str() );
    attron ( COLOR_PAIR ( 1 ) );
    mvprintw ( rows-1, cols-20, "press 'Esc' for quit" );
    attron ( COLOR_PAIR ( 1 ) );
    int firstRow = 3;
    while ( params.close == false ) {           // Don't echo() while we do getch
        std::stringstream message;
        std::stringstream message_error;
        getmaxyx ( stdscr,rows,cols );
	int row = firstRow;
	int col = 0;
	int offest = 20;
        mvprintw ( row, col+=offest, "vx: %lf", twist().linear.x);
        mvprintw ( row, col+=offest, "vy: %lf", twist().linear.y);
        mvprintw ( row, col+=offest, "vz: %lf", twist().linear.z);
        mvprintw ( row, col+=offest, "wx: %lf", twist().angular.x);
        mvprintw ( row, col+=offest, "wy: %lf", twist().angular.y);
        mvprintw ( row, col+=offest, "wz: %lf", twist().angular.z);
	ShmFw::Twist vel(twist());
        timeout ( 100 );
        key = getch();
        switch ( key ) {
        case KEY_UP:
            vel.linear.x += 0.1;
	    twist.set(vel);
            break;
        case KEY_DOWN:
            vel.linear.x -= 0.1;
	    twist.set(vel);
            break;
        case KEY_LEFT:
            vel.angular.z += 0.1;
	    twist.set(vel);
            break;
        case KEY_RIGHT:
            vel.angular.z -= 0.1;
	    twist.set(vel);
            break;
        case 's':
	    twist.set(ShmFw::Twist());
            break;
        case 27: //Esc
            params.close = true;
            break;
        }
    }
    endwin();
    exit ( 0 );
}



