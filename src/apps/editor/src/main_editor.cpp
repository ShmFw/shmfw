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
#include "shmfw/handler_variable.h"
#include "shmfw/deque.h"
#include "shmfw/log.h"
#include <ncurses.h>


SHMFW_INIT_LOG;

#include <boost/program_options.hpp>
#include <boost/thread.hpp>

size_t counter;
size_t activeParameter;

struct Prarmeters {
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    unsigned int timeout;
    bool dummy;
    bool info;
    bool rowview;
    std::string dummy_type;
    std::string dummy_value;
    std::vector<std::string> variable_names;
    bool close;
};

Prarmeters readArgs ( int argc, char **argv ) {
    namespace po = boost::program_options;

    Prarmeters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "info,i", "show shm info" )
    ( "rowview,r", "row view of containers instead of ; split" )
    ( "timeout,t", po::value<unsigned int> ( &params.timeout )->default_value ( 100 ), "update cycle time in ms, on 0 autoupdate off" )
    ( "dummy", "creates a dummy variable with the variable_names as name" )
    ( "dummy_type", po::value<std::string> ( &params.dummy_type )->default_value ( "double" ) , "defines the type of the dummy variable (double, float, int)" )
    ( "dummy_value", po::value<std::string> ( &params.dummy_value )->default_value ( "0" ) , "defines the value of the dummy variable" )
    ( "variable_name,n", po::value<std::vector<std::string> > ( &params.variable_names ), "shared variable names" )
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

    if ( vm.count ( "info" ) )  {
        params.info = true;
    } else {
        params.info = false;
    }
    if ( vm.count ( "rowview" ) )  {
        params.rowview = true;
    } else {
        params.rowview = false;
    }
    if ( vm.count ( "dummy" ) )  {
        params.dummy = true;
    } else {
        params.dummy = false;
    }
    if ( vm.count ( "help" ) )  {
        std::cout << desc << std::endl;
        exit ( 1 );
    }
    return params;
}


void print_help ( int rows, int row ) {
    move ( row, 0 );
    for ( int r = row; r < rows; r++ ) {
        move ( r, 0 );
        clrtoeol();
    }
    int r = row;
    attron ( COLOR_PAIR ( 1 ) );
    mvprintw ( r++, 5, "'Esc'   for exit" );
    mvprintw ( r++, 5, "'i'     for shm information" );
    mvprintw ( r++, 5, "'d'     row view of containers instead of ; split" );
    mvprintw ( r++, 5, "'h'     for help" );
    mvprintw ( r++, 5, "'t'     trigger a change" );
    mvprintw ( r++, 5, "'r'     remove lock" );
    mvprintw ( r++, 5, "'l'     lock" );
    mvprintw ( r++, 5, "'a'     toggle auto update" );
    mvprintw ( r++, 5, "'u'     for update" );
    mvprintw ( r++, 5, "'ENTER' to edit value" );
    mvprintw ( r++, 5, "any key to continue" );
    getch();
    for ( int r = row; r < rows; r++ ) {
        move ( r, 0 );
        clrtoeol();
    }
}


void printVariable ( const Prarmeters &param, std::vector < ShmFw::HandlerObjectPtr > &objects, int firstRow, int lastRow ) {
    int rows, cols;
    getmaxyx ( stdscr,rows,cols );
    for ( int row = firstRow; row < rows-2; row++ ) {
        move ( row, 0 );
        clrtoeol();
    }
    int row = firstRow;
    for ( size_t i = 0; i < objects.size(); i++ ) {
        if ( i == activeParameter ) attron ( COLOR_PAIR ( 2 ) );
        else attron ( COLOR_PAIR ( 1 ) );
        if ( param.info ) {
            mvprintw ( row++, 0, "%15s = %s, %s", objects[i]->name().c_str(), objects[i]->timestamp().c_str(), ( objects[i]->locked() ? "locked  " : "unlocked" ) );
        }
        if( param.rowview ){
	  for(uint32_t r = 0; r < objects[i]->size(); r++){
	    mvprintw ( row++, 0, "%15s[%2d] =%s %s", objects[i]->name().c_str(), r, ( objects[i]->locked() ? "*" : " " ), objects[i]->value(r).c_str() );
	  }
	} else {
	  mvprintw ( row++, 0, "%15s =%s %s", objects[i]->name().c_str(), ( objects[i]->locked() ? "*" : " " ), objects[i]->value().c_str() );
	}
    }
    mvprintw ( rows-2, cols-15, "updates: %d", counter++ );
}

int main ( int argc, char *argv[] ) {

    Prarmeters params = readArgs ( argc, argv );
    params.close = false;

    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    if ( params.dummy ) {
        for ( size_t i = 0; i < params.variable_names.size(); i++ ) {
            ShmFw::HandlerObjectPtr dummyHdlVar = ShmFw::HandlerObjectPtr ( ShmFw::HandlerObject::create ( params.variable_names[i], shmHdl, params.dummy_type ) );
            dummyHdlVar->value ( params.dummy_value );
        }
    }

    std::vector < ShmFw::HandlerObjectPtr > objects;
    if ( params.variable_names.size() == 0 ) {
        shmHdl->listNames ( params.variable_names );
    }

    for ( size_t i = 0; i < params.variable_names.size(); i++ ) {
        if ( shmHdl->findName ( params.variable_names[i] ) == NULL ) {
            std::cerr << "No variable: " << params.variable_names[i];
            std::cerr << " in share segment " << params.shm_memory_name;
            std::cerr << "!" << std::endl;
            return 0;
        }
        ShmFw::HandlerObjectPtr p = ShmFw::HandlerObject::open ( params.variable_names[i], shmHdl );
        if ( p ) {
            objects.push_back ( p );
        }
    }
    counter = 0;

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
    mvprintw ( 0, cols/2-5, "Editor" );
    attron ( COLOR_PAIR ( 1 ) );
    mvprintw ( 0, cols-20, "press 'h' for help" );
    attron ( COLOR_PAIR ( 1 ) );
    int firstParameterRow = 2;
    char str[80];
    std::string step_size_str;
    activeParameter = 0;
    bool auto_update = (params.timeout != 0);
    while ( params.close == false ) {           // Don't echo() while we do getch
        std::stringstream message;
        std::stringstream message_error;
        getmaxyx ( stdscr,rows,cols );
        printVariable ( params, objects, firstParameterRow, rows-3 );
	if(auto_update) timeout ( params.timeout );
	else timeout ( -1 );
        key = getch();
        if ( key == -1 ) continue;
        timeout ( -1 );
        move ( rows-1, 0 );
        clrtoeol();
        move ( rows-2, 0 );
        clrtoeol();
        switch ( key ) {
        case 'h':
            print_help ( rows, firstParameterRow );
            break;
        case 27: //Esc
            params.close = true;
            break;
        case '\n': //Enter
            echo();
            attron ( COLOR_PAIR ( 1 ) );
            mvprintw ( rows-1, 0, "%s = ", objects[activeParameter]->name().c_str() );
            getstr ( str );
            if ( strlen ( str ) > 0 ) {
                objects[activeParameter]->value ( str );
            }
            move ( rows-1, 0 );
            clrtoeol();
            noecho();
            break;
        case KEY_DOWN:
            if ( activeParameter < objects.size()-1 ) activeParameter++;
            break;
        case KEY_UP:
            if ( activeParameter > 0 ) activeParameter--;
            break;
        case 'r':
            objects[activeParameter]->unlock();
            message << "removed lock!";
            break;
        case 'l':
            objects[activeParameter]->lock();
            message << "lock!";
            break;
        case 't':
            objects[activeParameter]->it_has_changed();
            message << "change triggerd!";
            break;
        case 'u':
	    if(auto_update) auto_update = false;
            message << "press 'a' to activeate auto update or 'u' for refresh!";
            break;
        case 'a':
	    auto_update = !auto_update;
	    if(auto_update && (params.timeout == 0)) params.timeout = 100;
            message << "autoupdate: " << (auto_update?"on":"off") << "!";
            break;
        case 'd':
            params.rowview = !params.rowview;
            message << "toggel info!";
            break;
        case 'i':
            params.info = !params.info;
            message << "toggel info!";
            break;
        case ' ':
            break;
            message << "Press 'h' for help, and Esc for exit!";
        default:
            message << "Unkown key (" << key << "), press 'h' for help, and Esc for exit!";
        }
        attron ( COLOR_PAIR ( 1 ) );
        mvprintw ( rows-1, 0, "%s", message.str().c_str() );
        std::string error_str = message_error.str();
        if ( !error_str.empty() ) {
            attron ( COLOR_PAIR ( 5 ) );
            mvprintw ( rows-2, cols/2-error_str.length() /2, "%s", error_str.c_str() );
        }
        move ( rows-1, 0 );
    };
    endwin();
    exit ( 0 );

}



