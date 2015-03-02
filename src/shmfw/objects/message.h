/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2014 by Markus Bader <markus.bader@tuwien.ac.at>        *
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
#ifndef SHARED_MEM_OBJECTS_MESSAGE_H
#define SHARED_MEM_OBJECTS_MESSAGE_H

namespace ShmFw {

class Log;

class Message {
    friend class Log;
public:
    static const unsigned int MAX_MESSAGE_LENGTH = 64;
    enum Types {
        NA = 0,
        Info = 1,
        Debug = 2,
        Warning = 3,
        Error = 4,
        Critical = 5,
    };
private:
    int type;                        /// message type (level)
    int source;                      /// source to filter messages
    boost::posix_time::ptime time;
    char msg[MAX_MESSAGE_LENGTH];
    void updateMsg ( const std::string &_msg ) {
        unsigned int nrOfCharToCopy = _msg.length();
        if ( nrOfCharToCopy < MAX_MESSAGE_LENGTH ) {
            strncpy ( msg, _msg.c_str(), nrOfCharToCopy+1 );
        } else {
            nrOfCharToCopy = MAX_MESSAGE_LENGTH-5;
            strncpy ( msg, _msg.c_str(), nrOfCharToCopy );
            msg[nrOfCharToCopy-4] = ' ';
            msg[nrOfCharToCopy-3] = '.';
            msg[nrOfCharToCopy-2] = '.';
            msg[nrOfCharToCopy-1] = '.';
            msg[nrOfCharToCopy] = '\0';
        }
    }
    void updateMsg ( const char *format, va_list args ) {
        vsprintf ( msg, format, args );
    }
    void updateHeader ( int _type=NA, int _source=NA ) {
        type = _type;
        source = _source;
        time = boost::posix_time::microsec_clock::local_time();
    }
public:
    Message ()  {
    }
    Message ( int _type, int _source, const char *_format, ... ) {
        va_list args;
        va_start ( args, _format );
        updateHeader ( _type, _source );
        updateMsg ( _format,  args );
        va_end ( args );
    }
    Message ( const char *_format, ... ) {
        va_list args;
        va_start ( args, _format );
        updateHeader ( NA, NA );
        updateMsg ( _format,  args );
        va_end ( args );
    }
    Message ( int _type, int _source, const std::string &_msg ) {
        set ( _type, _source, _msg );
    }
    Message ( const std::string &_msg ) {
        set ( NA, NA, _msg );
    }
    void set ( int _type, int _source, const std::string &_msg ) {
        updateHeader ( _type, _source );
        updateMsg ( _msg );
    }
    void set ( const std::string &_msg ) {
        set ( NA, NA, _msg );
    }
    void set ( int _type, int _source, const char *_format, ... ) {
        va_list args;
        va_start ( args, _format );
        updateHeader ( _type, _source );
        updateMsg ( _format,  args );
        va_end ( args );
    }
    void set ( const char *_format, ... ) {
        va_list args;
        va_start ( args, _format );
        updateHeader ( NA, NA );
        updateMsg ( _format,  args );
        va_end ( args );
    }
    const boost::posix_time::ptime& getTime() const {
        return time;
    }
    int getType() const {
        return type;
    }
    int getSource() const {
        return source;
    }
    const char* getMsg() const {
        return msg;
    }
    /** equal operator **/
    bool operator == ( const Message m ) const {
        return time == m.time && type == m.type && ( strcmp ( msg, m.msg ) == 0 );
    }
    /** Creats a human readable string **/
    std::string human_readable() const {
        std::stringstream ss;
        ss << std::setw ( 3 ) << type << ", " << boost::posix_time::to_simple_string ( time ) <<  ", " << msg;
        return ss.str();
    }

    /** returns a string to the current type
     * @return type string
     **/
    std::string typeStr() const {
        switch ( type ) {
        case Info:
            return "Info";
        case Debug:
            return "Debug";
        case Warning:
            return "Warning";
        case Error:
            return "Error";
        case Critical:
            return "Critical";
        case NA:
            return "NA";
        default:
            return "?" ;
        };
    }
    /** reads a human readable string [ x, y, z]
     * @param str string to decipher
     * @param comma string ","
     * @param bracketOpen string "["
     * @param bracketClose string "]"
     * @return true on suggess
     **/
    bool human_readable ( const std::string &str, std::string comma = ",", std::string bracketOpen = "[", std::string bracketClose = "]" ) {
        if ( str.empty() ) return 1;
        return true;
    }
    friend std::ostream& operator<< ( std::ostream &os,const Message &o ) {
        os << boost::posix_time::to_simple_string ( o.time );
        if ( o.getType() != NA ) {
            os << ": " <<  std::setw ( 8 ) << o.typeStr();
        }
        os <<  ": " << o.msg;
        return os;
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "type", type );
        ar & make_nvp ( "source", source );
        ar & make_nvp ( "time", time );
        if ( archive::is_saving::value ) {
            std::string str ( msg );
            ar & make_nvp ( "msg", str );
        }

        if ( archive::is_loading::value ) {
            std::string str;
            ar & make_nvp ( "msg", str );
            updateMsg ( str );
        }
    }
};
};
#endif //SHARED_MEM_OBJECTS_MESSAGE_H

