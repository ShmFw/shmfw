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

#ifndef SHARED_MEM_LOG
#define SHARED_MEM_LOG

#include <iostream>
#include <valarray>
#include <boost/serialization/vector.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>
#include <shmfw/deque.h>
#include <shmfw/serialization/deque.h>
#include <shmfw/objects/message.h>

namespace ShmFw {

inline std::string DEFAULT_LOG_SEGMENT_NAME() {
    return "ShmFwLog";
};
inline unsigned int DEFAULT_LOG_SEGMENT_SIZE() {
    return 16*1048576; //16MB;
};

class Log;
/** Class to store log messages
  * @ToDo it should be implemented to be moveable by boost::move
**/


class Log: public Deque<Message> {
    static const unsigned int MAX_LOG_MESSAGES = 100;
    int sourceID;
    bool screen_output;
public:
#if BOOST_VERSION / 100 % 1000 <= 40
    //typedef boost::interprocess_container::deque_base<ShmFw::Message, ShmFw::Deque<ShmFw::Message>::Allocator >::iterator Iterator;
    typedef boost::container::deque_base<ShmFw::Message, ShmFw::Deque<ShmFw::Message>::Allocator >::iterator Iterator;
#else
    typedef boost::container::deque_base<ShmFw::Message, ShmFw::Deque<ShmFw::Message>::Allocator >::iterator Iterator;
#endif

    /**
     * Enables or disables the screen output
     * @param screen on true it sets a local output on false only the logger shows the output
     **/
    void screenOutput ( bool screen ) {
        screen_output = screen;
    }

    /**
     * @ToDo sets the name of the logger
     **/
    void setSourceName ( unsigned int  i ) {
        /// create a shared vector of strings and use the index for the source
        sourceID = i;
    }
    void write ( const Message &msg, bool trigger = true ) {
        if ( screen_output && ( msg.getType() >= ( int ) process_cout_level() ) ) {
            if ( msg.getType() < Message::Error ) {
                std::cout << msg << std::endl;
            } else {
                std::cerr << msg << std::endl;
            }
        }
        lock();
        if ( size() > MAX_LOG_MESSAGES ) pop_front();
        push_back ( msg );
        if ( trigger ) itHasChanged();
        unlock();
    }
    void write ( int type, const std::string &msg, bool trigger = true ) {
        write ( Message ( type, sourceID, msg ), trigger );
    }
    void info ( const std::string &msg, bool trigger = true ) {
        write ( Message::Info, msg );
    }
    void warning ( const std::string &msg, bool trigger = true ) {
        write ( Message::Warning, msg );
    }
    void error ( const std::string &msg, bool trigger = true ) {
        write ( Message::Error, msg );
    }
    void critcal ( const std::string &msg, bool trigger = true ) {
        write ( Message::Critical, msg );
    }
    void debug ( const std::string &msg, bool trigger = true ) {
        write ( Message::Debug, msg );
    }
    friend std::ostream &operator << ( std::ostream &os, const ShmFw::Log &o ) {
        for ( unsigned int i = 0; i < o.size(); i++ ) {
            os << o.at ( i ) << std::endl;
        }
        return os;
    };
    void process_cout_level ( unsigned int value ) {
        userRegister ( value );
    }
    unsigned int process_cout_level () {
        return userRegister ();
    }
    /** Write into a file **/
    void write ( const std::string &filename ) {
        std::ofstream ofs ( filename.c_str() );
        assert ( ofs.good() );
        boost::archive::xml_oarchive xml ( ofs );
        xml << boost::serialization::make_nvp ( "Vec", *this );
    }
    /** Read from a file **/
    void read ( const std::string &filename ) {
        std::ifstream ifs ( filename.c_str() );
        assert ( ifs.good() );
        boost::archive::xml_iarchive xml ( ifs );
        xml >> boost::serialization::make_nvp ( "Vec", *this );
    }

    /** Constructor
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment
     * @param screen enables or disable local screen output
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     **/
    Log ( HandlerPtr &shmHdl, const std::string &name = "log", bool screen = true )
        : Deque<Message> ( name, shmHdl )
        , sourceID ( Message::NA )
        , screen_output ( screen ) {
    }

protected:
    Log();
    Log ( Log const& );           // Don't Implement
    void operator= ( Log const& ); // Don't implement
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        if ( archive::is_saving::value ) {
            std::vector<ShmFw::Message> tmp;
            get ( tmp );
            ar & make_nvp ( "msg", tmp );
        }
        if ( archive::is_loading::value ) {
            std::vector<Message> tmp;
            ar & make_nvp ( "msg", tmp );
            set ( tmp );
        }
    }
};
};

#define SHMFW_INIT_LOG  \
    ShmFw::HandlerPtr _shmHdlLog = ShmFw::Handler::create( ShmFw::DEFAULT_LOG_SEGMENT_NAME(), ShmFw::DEFAULT_LOG_SEGMENT_SIZE() );\
    ShmFw::Log _log( _shmHdlLog , "log", false );
#define SHMFW_LOG_SCREEN_OUPUT (screen) _log.screenOutput( screen );

#define SHMFW_Info(message)  _log.write(ShmFw::Message(ShmFw::Message::Info, 0, "%s", message), true);
#define SHMFW_Info_FNC(message)  _log.write(ShmFw::Message(ShmFw::Message::Info, 0, "%s: %s", __FUNCTION__, message), true);
#define SHMFW_Debug(message)  _log.write(ShmFw::Message(ShmFw::Message::Debug, 0, "%s", message), true);
#define SHMFW_Debug_FNC(message)  _log.write(ShmFw::Message(ShmFw::Message::Debug, 0, "%s: %s", __FUNCTION__, message), true);
#define SHMFW_Warning(message)  _log.write(ShmFw::Message(ShmFw::Message::Warning, 0, "%s", message), true);
#define SHMFW_Warning_FNC(message)  _log.write(ShmFw::Message(ShmFw::Message::Warning, 0, "%s: %s", __FUNCTION__, message), true);
#define SHMFW_Error(message)  _log.write(ShmFw::Message(ShmFw::Message::Error, 0, "%s", message), true);
#define SHMFW_Error_FNC(message)  _log.write(ShmFw::Message(ShmFw::Message::Error, 0, "%s: %s", __FUNCTION__, message), true);
#define SHMFW_Critical(message)  _log.write(ShmFw::Message(ShmFw::Message::Critical, 0, "%s", message), true);
#define SHMFW_Critical_FNC(message)  _log.write(ShmFw::Message(ShmFw::Message::Critical, 0, "%s: %s", __FUNCTION__, message), true);


#endif //SHARED_MEM_LOG


