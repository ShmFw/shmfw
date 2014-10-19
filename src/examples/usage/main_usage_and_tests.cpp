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

#include <shmfw/objects/pose.h>
#include <shmfw/objects/pose2d.h>
#include <shmfw/objects/pose2d_agv.h>
#include <shmfw/objects/point.h>
#include <shmfw/objects/quaternion.h>
#include <shmfw/variable.h>
#include <shmfw/vector.h>
#include <shmfw/deque.h>
#include <shmfw/log.h>
#include <shmfw/serialization/variable.h>
#include <shmfw/serialization/io_file.h>


SHMFW_INIT_LOG;

#include <boost/program_options.hpp>
#include <boost/thread.hpp>

struct Prarmeters {
    bool clear;
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
    ( "clear,c", "clears the shared memory" )
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
    params.clear = ( vm.count ( "clear" ) > 0 );

    return params;
}

void conditionThread ( ShmFw::HandlerPtr &shmHdl, const std::string &name ) {

    SHMFW_Info_FNC ( "started" );
    ShmFw::Vector<int> xy ( name, shmHdl );
    while ( xy.timed_wait ( 500 ) == false ) SHMFW_Debug_FNC ( "waiting 500ms" );
    xy.lock();
    xy.push_back ( 100 );
    std::cout << xy.human_readable() << std::endl;
    xy.unlock();
    xy.itHasChanged();
    SHMFW_Info_FNC ( "finished" );
}

int main ( int argc, char *argv[] ) {


    Prarmeters params = readArgs ( argc, argv );
    if ( params.clear ) {
        ShmFw::Handler::removeSegment ( params.shm_memory_name );
        std::cout << "Shared Memory " << params.shm_memory_name << " cleared" << std::endl;
    }
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( params.shm_memory_name, params.shm_memory_size );

    ShmFw::Var<double> a ( "a", shmHdl, 3 );
    a.set ( 5.4 );
    std::cout << a.info_shm();
    std::cout << a.human_readable() << std::endl;
    a() = 1.2;
    std::cout << a.human_readable() << std::endl;

    ShmFw::Var<double> aa ( "a", shmHdl, 3 );
    std::cout << a.human_readable() << std::endl;

    ShmFw::Vector<double> b ( "b", shmHdl );
    std::cout << b.info_shm();
    b().push_back ( 5 );
    b().push_back ( 3 );
    b[0] = 0.3;
    std::cout << b.human_readable() << std::endl;

    ShmFw::Var<ShmFw::Point > point ( "point", shmHdl );
    std::cout << "point: " << point() << std::endl;
    point().setFromString ( "[43,44.4,2.4]" );
    std::cout << "point: " << point() << std::endl;

    ShmFw::Var<ShmFw::Quaternion > quaternion ( "quaternion", shmHdl );
    std::cout << "quaternion: " << quaternion() << std::endl;
    quaternion().setFromString ( "[0.1,4,35,43.2]" );
    std::cout << "quaternion: " << quaternion() << std::endl;

    ShmFw::Var<ShmFw::Pose > pose ( "pose", shmHdl );
    std::cout << "pose: " << pose() << std::endl;
    pose().setFromString ( "[[42,4,2.6],[22,0.3,03, 2]]" );
    std::cout << "pose: " << pose() << std::endl;

    ShmFw::Var<ShmFw::Pose2D > pose2d ( "pose2d", shmHdl );
    std::cout << "pose2d: " << pose2d() << std::endl;
    pose2d().setFromString ( "[[45.2,8.4],[3.14]]" );
    std::cout << "pose2d: " << pose2d() << std::endl;

    ShmFw::Var<ShmFw::Pose2DAGV > pose2d_agv ( "pose2d_agv", shmHdl );
    std::cout << "pose2d_agv: " << pose2d_agv() << std::endl;
    pose2d_agv().setFromString ( "[[45.2,8.4],[3.14], [3]]" );
    std::cout << "pose2d_agv: " << pose2d_agv() << std::endl;

    try {
        ShmFw::Var<double> c ( "c", shmHdl, 1 );
        c = 2;
        std::cout << "This will produces an error: " << c.human_readable()  << " = " << a.human_readable()  << std::endl;
        c = a;
        std::cout << c.human_readable() << std::endl;
    } catch ( std::runtime_error err ) {
        std::cout << err.what() << std::endl;
    }

    int testInt = std::rand() % 10;
    {
        ShmFw::Var<int> xy ( "myInt", shmHdl,1 );
        xy = testInt;
        ShmFw::write ( "test.xml", xy, ShmFw::FORMAT_XML );
        std::cout << "wrote: xy: " << xy.human_readable()  << std::endl;
    }

    {
        ShmFw::Var<int> xy ( "Other", shmHdl,1 );
        std::cout << "goint to read xy: " << std::endl;
        ShmFw::read ( "test.xml", xy, ShmFw::FORMAT_XML );
        if ( xy() != testInt ) std::cout << "problem!"  << std::endl;
        std::cout << "read xy: " << xy.human_readable()  << std::endl;
    }

    ShmFw::Vector<double> d ( "d", shmHdl );
    d().push_back ( 4.3 );
    std::cout << d.human_readable() << std::endl;

    ShmFw::VectorStr log ( "log", shmHdl );
    ShmFw::CharAllocator allocator ( shmHdl->getShm()->get_segment_manager() );
    ShmFw::CharString mystring ( allocator );
    mystring = "Hello";
    log().push_back ( mystring );
    ShmFw::CharString xx = shmHdl->createString();
    xx = "-";
    log().push_back ( xx );
    log().push_back ( shmHdl->createString ( "World" ) );
    mystring = "!";
    log().push_back ( mystring );
    log.push_back ( " How are you" );
    std::cout << log.human_readable() << std::endl;


    log.destroy();
    //a.destroy();

    std::string varName ( "xy" );
    boost::thread t1 ( conditionThread, shmHdl,  varName );
    ShmFw::Vector<int> xy ( varName, shmHdl );
    xy().clear();
    sleep ( 2 );
    xy.lock();
    xy.push_back ( 3 );
    xy.unlock();
    xy.itHasChanged();
    xy.wait();

    b.unlock();
    b.lock();
    std::cout << "timed_lock, waits to get the lock 1sec" << std::endl;
    if ( b.timed_lock ( 1000 ) == false ) std::cout << "timeout" << std::endl;
    std::cout << "timed_wait, waits for contition/signal 1sec" << std::endl;
    if ( b.timed_wait ( 1000 ) == false ) std::cout << "timeout" << std::endl;
    b.unlock();
    std::cout << "exit" << std::endl;

    ShmFw::Deque<int> myDeque0 ( "myDeque", shmHdl );
    myDeque0().clear();
    myDeque0.push_back ( 3 );
    myDeque0.push_back ( 4 );
    myDeque0.push_back ( 6 );
    ShmFw::Deque<int> myDeque1 ( "myDeque", shmHdl );
    myDeque0().clear();
    myDeque1.push_front ( 3 );
    myDeque0.push_back ( 4 );
    myDeque0.push_back ( 6 );

    ShmFw::Deque<ShmFw::Pose2D> waypoints ( "waypoints", shmHdl );
    waypoints().clear();
    waypoints.push_back ( ShmFw::Pose2D ( ShmFw::Pose2D ( 1, 2, 0.3 ) ) );
    waypoints.push_back ( ShmFw::Pose2D ( ShmFw::Pose2D ( 2, 3, 0.1 ) ) );


    exit ( 0 );

}
