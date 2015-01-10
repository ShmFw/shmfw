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


#include <opencv2/highgui/highgui.hpp>
#include <shmfw/allocator.h>
#include <shmfw/variable.h>
#include <shmfw/objects/grid_map.h>
#include <shmfw/objects/opencv.h>
#include <boost/program_options.hpp>

struct Prarmeters {
    int reload;
    std::string  cvtype;
    double grid;
    bool on_change;
    int border;
    std::string shm_memory_name;
    unsigned int shm_memory_size;
    std::string variable_name;
};

Prarmeters readArgs ( int argc, char **argv ) {
    namespace po = boost::program_options;

    Prarmeters params;
    po::options_description desc ( "Allowed Parameters" );
    desc.add_options()
    ( "help", "get this help message" )
    ( "on_change,c", "updates only on change" )
    ( "border", po::value< int > ( &params.border )->default_value ( 20 ), "view border" )
    ( "reload,r", po::value< int > ( &params.reload )->default_value ( 100 ), "reload time in ms, 0 means reload on signal, -1 means relaod owns to load image into shm" )
    ( "cvtype,t", po::value<std::string> ( &params.cvtype ), "opencv type: CV_8UC3, CV_8U, ..." )
    ( "grid,g", po::value< double > ( &params.grid )->default_value ( 1.0 ), "grid size, -1 means no grid" )
    ( "shm_memory_name,m", po::value<std::string> ( &params.shm_memory_name )->default_value ( ShmFw::DEFAULT_SEGMENT_NAME() ), "shared memory segment name" )
    ( "shm_memory_size,s", po::value<unsigned int> ( &params.shm_memory_size )->default_value ( ShmFw::DEFAULT_SEGMENT_SIZE() ), "shared memory segment size" )
    ( "variable_name,v", po::value<std::string> ( &params.variable_name )->default_value ( "image_grid" ), "shared variable name" );

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
    params.on_change = (vm.count ( "on_change" ) > 0); 

    return params;
}

bool loop_program = true;

void terminate ( int param ) {
    std::cout << "Program was trying to abort or terminate." << std::endl;
    loop_program = false;
}

void draw_grid ( const Prarmeters &params,  const ShmFw::Var<ShmFw::GridMapInterprocess<uchar> > &grid,  cv::Mat &des ) {
    cv::Scalar colorOrigin ( 0,0,0 );
    cv::Scalar colorGrid ( 200,200,200 );
    cv::Scalar colorText ( 0,0,0 );
    double textHeight = 0.3;
    int xc = grid->x2idx ( 0 ) + params.border;
    int yc = grid->y2idx ( 0 ) + params.border;
    char text[0xFF];
    {
        /// vertical
        double x_max = round ( grid->getXMax() / params.grid );
        double x_min = round ( grid->getXMin() / params.grid );
        for ( double x0 = x_min; x0 <= x_max; x0+= params.grid ) {
            int x = grid->x2idx ( x0 )  + params.border;
            cv::line ( des, cv::Point ( x, 0 ), cv::Point ( x, des.rows ), colorGrid, 1 );
            if ( ( x0 != x_min ) && ( x0 != x_max ) ) {
		sprintf(text, "%4.2f", x0);
                //cv::putText ( des, boost::lexical_cast<std::string> ( x0 ), cv::Point (   x, yc+2 ), cv::FONT_HERSHEY_SIMPLEX, textHeight, colorText, 1, CV_AA );
                cv::putText ( des, text, cv::Point ( x+2, des.rows-10 ), cv::FONT_HERSHEY_SIMPLEX, textHeight, colorText, 1, CV_AA );
            }
        }
        cv::putText ( des, "Y", cv::Point ( 10, 10 ), cv::FONT_HERSHEY_SIMPLEX, textHeight, colorText, 1, CV_AA );
    }
    {
        /// horizontal
        double y_max = round ( grid->getYMax() / params.grid );
        double y_min = round ( grid->getYMin() / params.grid );
        for ( double y0 = y_min; y0 <= y_max; y0+= params.grid ) {
            int y = grid->y2idx ( y0 ) + params.border;
            cv::line ( des, cv::Point ( 0, y ), cv::Point ( des.cols, y ), colorGrid, 1 );
            if ( ( y0 != y_min ) && ( y0 != y_max ) ) {
		sprintf(text, "%4.2f", y0);
                //cv::putText ( des, boost::lexical_cast<std::string> ( y0 ), cv::Point ( xc+2, y+2 ), cv::FONT_HERSHEY_SIMPLEX, textHeight, colorText, 1, CV_AA );
                cv::putText ( des, text, cv::Point ( 2, y-6 ), cv::FONT_HERSHEY_SIMPLEX, textHeight, colorText, 1, CV_AA );
            }
        }
        cv::putText ( des, "X", cv::Point ( des.cols-10, des.rows-10 ), cv::FONT_HERSHEY_SIMPLEX, textHeight, colorText, 1, CV_AA );

    }
    {
        /// origin
        cv::line ( des, cv::Point ( xc, 0 ), cv::Point ( xc, des.rows ), colorOrigin, 2 );
        cv::line ( des, cv::Point ( 0, yc ), cv::Point ( des.cols, yc ), colorOrigin, 2 );
    }

}
void view_image ( const Prarmeters &params, ShmFw::HandlerPtr &shmHdl ) {
    ShmFw::Var<ShmFw::GridMapInterprocess<uchar> > grid ( params.variable_name, shmHdl );
    std::cout << *grid << std::endl;
    int cvtype = -1;
    if ( params.cvtype.empty() ) {
        if ( grid->getDepth() == 1 ) cvtype = CV_8U;
        else if ( grid->getDepth() == 2 ) cvtype = CV_16U;
        else if ( grid->getDepth() == 3 ) cvtype = CV_8UC3;
    } else {
        cvtype = cv::solveCvType ( params.cvtype );
    }
    if ( cvtype == -1 ) {
        std::cerr << "could not detect image cvtype!" << std::endl;
    }

    int key;
    cv::Mat img;
    cv::Mat des;    
    do {
        cv::Mat src = grid->cvMat ( cvtype );
        img.create ( src.rows+params.border*2, src.cols + params.border*2, cvtype );
        img.setTo ( 0xFF );
        des = cv::Mat ( img, cv::Rect ( params.border,params.border, src.cols, src.rows ) );
        src.copyTo ( des );
        if ( params.grid > 0 ) {
            draw_grid ( params,  grid, img );
        }
        cv::imshow ( params.variable_name.c_str(), img );
	if(params.on_change){
	  key = cv::waitKey ( params.reload );
	  while(grid.timed_wait(10000) == false){
	    boost::posix_time::time_duration d = ShmFw::now() - grid.timestampShm();
	    std::cout << "Last change " << d.total_milliseconds() << " ms ago!" << std::endl;
	  }
	} else {
	  key = cv::waitKey ( params.reload );
	}
    } while ( loop_program && ( key < 0 ) );
    cv::destroyWindow ( params.variable_name.c_str() );
}
int main ( int argc, char **argv ) {

    signal ( SIGABRT,	terminate );
    signal ( SIGTERM,	terminate );
    Prarmeters params = readArgs ( argc, argv );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );
    ShmFw::Header *header = ( ShmFw::Header * ) shmHdl->findName ( params.variable_name );
    if ( header ) {
        view_image ( params, shmHdl );
    }


    exit ( 0 );
}
