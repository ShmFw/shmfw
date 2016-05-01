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

#ifndef SHARED_MEM_HANDLER_OBJECT
#define SHARED_MEM_HANDLER_OBJECT

#include <shmfw/variable.h>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>

namespace ShmFw {

class HandlerObject;

typedef std::shared_ptr<HandlerObject> HandlerObjectPtr;

/**
 * class to enable polymorphismus on shared objects 
 **/
class HandlerObject {
public:
    virtual std::string name() {
        return "xx";
    }
    virtual std::string type_name() {
        return "xx";
    }
    virtual void it_has_changed() {
    }
    virtual void lock() {
    }
    virtual void unlock() {
    }
    virtual bool locked() {
      return false;
    }
    virtual std::string timestamp() {
        return "xx";
    }
    virtual std::string value() const {
        return "xx";
    }
    virtual std::string value(uint32_t i) const {
        return "xx";
    }
    virtual uint32_t size() const {
        return 0;
    }
    virtual void value(const std::string &str) {
    }
    virtual int construct ( const std::string &name, HandlerPtr &shmHdl, unsigned int size = 1 ){
      return ShmFw::Header::ERROR;
    }
    static HandlerObjectPtr open(const std::string &name, HandlerPtr &shmHdl);
    static HandlerObjectPtr create(const std::string &name, HandlerPtr &shmHdl, const std::string &type);
};



};



#endif //SHARED_MEM_HANDLER_OBJECT




