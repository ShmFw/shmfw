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

#ifndef SHARED_MEM_FW_H
#define SHARED_MEM_FW_H

#include <string>
#include <boost/program_options/options_description.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

namespace ShmFw {

class Parameter;
typedef std::shared_ptr<ShmFw::Parameter> ParameterPtr;
/// class to handle shared memory parameters
class Parameter {
public:
    std::string segment_name;
    size_t segment_size;
    std::string segment_ns;  
    /**
     * constructor
     **/
    Parameter();
    /**
     * constructor
     * @param name name of the shared segment
     * @param  size  size of the shared segment
     * @param  ns namespace used to create variables
     **/
    Parameter ( const std::string& name, unsigned int size, const std::string& ns = "" );
    friend std::ostream &operator << ( std::ostream &os, const Parameter &o ) {
        os << "segment_name =  " << o.segment_name << std::endl;
        os << "segment_size =  " << o.segment_size << std::endl;
        os << "segment_ns =  " << o.segment_ns << std::endl;
        return os;
    }
    /**
     * resolves a name and addes the namespace
     * @param name name of the shared segment
     * @return resolved name
     **/
    std::string resolve_namespace ( const std::string &_name );
    
    /// fixes the namespace syntax
    void fix_namespace_syntax(); 
    /// creates a smart pointer to the class
    static ParameterPtr create();
    /** creates a smart pointer to the class
     * @param name name of the shared segment
     * @param  size  size of the shared segment
     * @param  ns namespace used to create variables
     **/
    static ParameterPtr create ( const std::string& name, unsigned int size, const std::string& ns = "" );
    /**
     * returns a boost program option description
     * @param group option group
     * @return program option description
     * @post you should execute fix_namespace_syntax() to fix the namespace format
     **/
    boost::program_options::options_description options (const std::string &group);
};


enum SerializeFormat {
    FORMAT_NA  = 0,
    FORMAT_XML = 1,
    FORMAT_BIN = 2,
    FORMAT_TXT = 3
};

boost::posix_time::ptime now();
std::string DEFAULT_SEGMENT_NAME();
size_t DEFAULT_SEGMENT_SIZE();

};
#endif //SHARED_MEM_FW_H

