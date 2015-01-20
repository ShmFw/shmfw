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
#include "shmfw/objects/handler_parameterentry.h"
#include "shmfw/deque.h"
#include "shmfw/log.h"
#include <ncurses.h>


SHMFW_INIT_LOG;

#include <boost/program_options.hpp>
#include <boost/thread.hpp>

struct Prarmeters {
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::string ns;
    std::vector<std::string> variable_name;
    bool close;
};

Prarmeters readArgs ( int argc, char **argv ) {
    namespace po = boost::program_options;

    Prarmeters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "namespace", po::value<std::string> ( &params.ns ), "shm namespace (robot name) " )
    ( "variable_name,n", po::value<std::vector<std::string> > ( &params.variable_name ), "shared variable name" )
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

    if ( vm.count ( "variable_name" ) == 0)  {
        std::cout << "missing argument: variable_name" << std::endl;
        std::cout << desc << std::endl;
        exit ( 1 );
    }
    if ( vm.count ( "help" ) )  {
        std::cout << desc << std::endl;
        exit ( 1 );
    }
    return params;
}

void print_help(int rows, int row) {
    move(row, 0);
    for(int r = row; r < rows; r++) {
        move(r, 0);
        clrtoeol();
    }
    int r = row;
    attron(COLOR_PAIR(1));
    mvprintw(r++, 5, "'Esc'   for exit");
    mvprintw(r++, 5, "'h'     for help");
    mvprintw(r++, 5, "'e'     for enable");
    mvprintw(r++, 5, "'d'     for disable");
    mvprintw(r++, 5, "'t'     for toggel");
    mvprintw(r++, 5, "'ENTER' to edit value");
    mvprintw(r++, 5, "'u'     to edit upper bound (max)");
    mvprintw(r++, 5, "'l'     to edit lower bound (max)");
    mvprintw(r++, 5, "'s'     to edit step size");
    mvprintw(r++, 5, "any key to continue");
    getch();
    for(int r = row; r < rows; r++) {
        move(r, 0);
        clrtoeol();
    }
}

int main ( int argc, char *argv[] ) {

    Prarmeters params = readArgs ( argc, argv );
    params.close = false;

    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create( params.shm_memory_name, params.shm_memory_size, params.ns );

    /*
            ShmFw::HandlerParameterBasePtr parameter = ShmFw::parameterEntryHdl("param_double", shmHdl);
            std::cout << parameter->getString() << std::endl;
            parameter->increase();
            std::cout << parameter->getString() << std::endl;
            parameter->decrease();
            std::cout << parameter->getString() << std::endl;
    */

    std::vector<ShmFw::HandlerParameterBasePtr> parameters;
    for(size_t i = 0; i < params.variable_name.size(); i++) {
        parameters.push_back(ShmFw::HandlerParameterBasePtr(ShmFw::HandlerParameterBase::open(params.variable_name[i], shmHdl)));
    }


    initscr();                        // Start curses mode
    raw();                            // Line buffering disabled
    start_color();
    init_pair(1, COLOR_WHITE, COLOR_BLACK);
    init_pair(1, COLOR_WHITE, COLOR_BLACK);
    init_pair(2, COLOR_RED, COLOR_BLACK);
    init_pair(3, COLOR_GREEN, COLOR_BLACK);
    init_pair(4, COLOR_BLUE, COLOR_BLACK);
    init_pair(5, COLOR_RED, COLOR_YELLOW);
    keypad ( stdscr, TRUE );          // We get F1, F2 etc..
    noecho();
    int key;
    int rows,cols;
    getmaxyx ( stdscr,rows,cols );
    attron(COLOR_PAIR(3));
    mvprintw ( 0, cols/2-5, "Reconfigure");
    attron(COLOR_PAIR(1));
    mvprintw ( 0, cols-20, "press 'h' for help");
    attron(COLOR_PAIR(1));
    int firstParameterRow = 2;
    size_t activeParameter = 0;
    char str[80];
    while(params.close == false) {              // Don't echo() while we do getch
        std::stringstream message;
        std::stringstream message_error;
        int parameterRow;
        getmaxyx ( stdscr,rows,cols );
        char *bar = new char[cols+1];
        for(size_t i = 0; i < parameters.size(); i++) {
            if(i == activeParameter) {
                attron(COLOR_PAIR(2));
            } else {
                attron(COLOR_PAIR(4));
            }
            parameterRow = firstParameterRow + i*2;
            move(parameterRow, 0);
            clrtoeol();
            mvprintw ( parameterRow, 0, "%15s = %s", params.variable_name[i].c_str(), parameters[i]->getString().c_str() );
            int pos_bar = parameters[i]->control_position() * cols;

            for(int c = 0; c < cols; c++) {
                bar[c] = ((c < pos_bar)?'#':' ');
            }
            bar[cols] = '\0';
            mvprintw ( parameterRow + 1, 0, "%s", bar );
            if(parameters[i]->valid() == false) {
                attron(COLOR_PAIR(5));
                mvprintw ( parameterRow + 1, cols/2-5, " invalid ");
            }
        }
        key = getch();
        move(rows-1, 0);
        clrtoeol();
        move(rows-2, 0);
        clrtoeol();
        switch ( key ) {
        case 27: //Esc
            params.close = true;
            break;
        case 'h':
            print_help(rows, firstParameterRow);
            break;
        case KEY_RIGHT:
            parameters[activeParameter]->increase();
            break;
        case KEY_LEFT:
            parameters[activeParameter]->decrease();
            break;
        case KEY_DOWN:
            if(activeParameter < parameters.size()-1) activeParameter++;
            break;
        case KEY_UP:
            if(activeParameter > 0) activeParameter--;
            break;
        case 't':
            parameters[activeParameter]->enable(!parameters[activeParameter]->enable());
            break;
        case 'e':
            parameters[activeParameter]->enable(true);
            break;
        case 'd':
            parameters[activeParameter]->enable(false);
            break;
        case '\n': //Enter
            echo();
            attron(COLOR_PAIR(1));
            mvprintw(rows-1, 0, "%s value = ", params.variable_name[activeParameter].c_str());
            getstr(str);
            move(rows-1, 0);
            clrtoeol();
            if(parameters[activeParameter]->setValueFromString(str) == false) {
                message_error << "incorrect format ";
            }
            noecho();
            break;
        case 's': 
            echo();
            attron(COLOR_PAIR(1));
            mvprintw(rows-1, 0, "%s step = ", params.variable_name[activeParameter].c_str());
            getstr(str);
            move(rows-1, 0);
            clrtoeol();
            if(parameters[activeParameter]->setStepFromString(str) == false) {
                message_error << "incorrect format ";
            }
            noecho();
            break;
        case 'u': 
            echo();
            attron(COLOR_PAIR(1));
            mvprintw(rows-1, 0, "%s upper bound = ", params.variable_name[activeParameter].c_str());
            getstr(str);
            move(rows-1, 0);
            clrtoeol();
            if(parameters[activeParameter]->setMaxFromString(str) == false) {
                message_error << "incorrect format ";
            }
            noecho();
            break;
        case 'l': 
            echo();
            attron(COLOR_PAIR(1));
            mvprintw(rows-1, 0, "%s lower bound = ", params.variable_name[activeParameter].c_str());
            getstr(str);
            move(rows-1, 0);
            clrtoeol();
            if(parameters[activeParameter]->setMaxFromString(str) == false) {
                message_error << "incorrect format ";
            }
            noecho();
            break;
        default:
            message << "Unkown key (" << key << "), press 'h' for help, and Esc for exit!";
        }
        attron(COLOR_PAIR(1));
        mvprintw ( rows-1, 0, "%s", message.str().c_str() );
        std::string error_str = message_error.str();
        if(!error_str.empty()) {
            attron(COLOR_PAIR(5));
            mvprintw ( rows-2, cols/2-error_str.length()/2, "%s", error_str.c_str() );
        }
        move(rows-1, 0);
        delete bar;
    } ;
    endwin();


    exit ( 0 );

}

