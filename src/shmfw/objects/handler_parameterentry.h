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

#ifndef SHARED_MEM_OBJECT_HANDLER_PARAMETERENTRY
#define SHARED_MEM_OBJECT_HANDLER_PARAMETERENTRY

#include <boost/lexical_cast.hpp>
#include <shmfw/objects/parameterentry.h>
#include <boost/shared_ptr.hpp>

namespace ShmFw {

class HandlerParameterBase {
public:
    virtual void decrease() {
        throw 0;
    }
    virtual void increase() {
        throw 0;
    }
    virtual bool valid() const {
        throw 0;
    }
    virtual bool enable(bool) {
        throw 0;
    }
    virtual bool enable() const {
        throw 0;
    }
    virtual std::string getString() {
        return "xx";
    }
    virtual bool setValueFromString(const std::string &) {
        throw 0;
    }
    virtual bool setStepFromString(const std::string &) {
        throw 0;
    }
    virtual bool setMinFromString(const std::string &) {
        throw 0;
    }
    virtual bool setMaxFromString(const std::string &) {
        throw 0;
    }
    virtual double control_position() const {
        return 0;
    }
    static HandlerParameterBase* open(const std::string &name, HandlerPtr &shmHdl);
};

typedef std::shared_ptr<HandlerParameterBase> HandlerParameterBasePtr;


template <class T>
class HandlerParameter : public HandlerParameterBase {
public:
    static const int TYPE_ASSIGNMENT_EXEPTION = 10;
    static const int TYPE_COMPARE_EXEPTION = 20;
    static const int TYPE_UNKOWN_EXEPTION = 30;
private:
    ShmFw::Var<ShmFw::ParameterEntry<T> >entry_;
public:
    HandlerParameter(const std::string &name, HandlerPtr &shmHdl)    {
        entry_.construct (name, shmHdl);
    }
    /** increases value one step
     * @return value
     **/
    void increase() {
        entry_->increase();
    }
    /** decrease value one step
     * @return value
     **/
    void decrease() {
        entry_->decrease();
    }
    /** checks if the parameter value is between min and max and enable
     * @return enable
     **/
    bool valid() const {
        return entry_->valid();
    }
    /** sets enable
     * @param new value
     * @return enable
     **/
    bool enable( bool _enable ) {
        return entry_->enable(_enable);
    }
    /** returns enable value
     * @return enable
     **/
    bool enable() const {
        return entry_->enable_;
    }
    /** control position
     * @return (max - min)/(value_ - min_)
     **/
    double control_position() const {
        double w = (double) (entry_->max_ - entry_->min_);
        double pos = (double) entry_->value_ - entry_->min_;
        return pos / w;
    }
    bool setValueFromString(const std::string &str) {
        try {
            entry_->value( boost::lexical_cast<T>(str));
        } catch(const boost::bad_lexical_cast &) {
            return false;
        }
        return true;
    }
    bool setStepFromString(const std::string &str) {
        try {
            entry_->step_size( boost::lexical_cast<T>(str));
        } catch(const boost::bad_lexical_cast &) {
            return false;
        }
        return true;
    }
    bool setMaxFromString(const std::string &str) {
        try {
            entry_->max( boost::lexical_cast<T>(str));
        } catch(const boost::bad_lexical_cast &) {
            return false;
        }
        return true;
    }
    bool setMinFromString(const std::string &str) {
        try {
            entry_->min( boost::lexical_cast<T>(str));
        } catch(const boost::bad_lexical_cast &) {
            return false;
        }
        return true;
    }
    std::string getString () {
        std::stringstream ss;
        ss << std::setw ( 8 ) << entry_->value_         << ", ";
        ss << std::setw ( 8 ) << entry_->min_           << ", ";
        ss << std::setw ( 8 ) << entry_->max_           << ", ";
        ss << ( entry_->enable_?"   enable":" disabled" ) << ", ";
        ss << std::setw ( 7 ) << entry_->step_size_     << ", ";
        return ss.str();
    };

};

    
HandlerParameterBase* HandlerParameterBase::open(const std::string &name, HandlerPtr &shmHdl) {

        if(shmHdl->findName(name) == NULL ) {
            std::cerr << "no shared variable with name: " << name << std::endl;
            throw 0;
        }

        ShmFw::Header shmHeader(shmHdl, name);

        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<float> > >()) return new ShmFw::HandlerParameter<float>( name, shmHdl);
        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<double> > >()) return new ShmFw::HandlerParameter<double>( name, shmHdl);

        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<bool> > >())    return new ShmFw::HandlerParameter<bool>( name, shmHdl);
        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<char> > >())    return new ShmFw::HandlerParameter<char>( name, shmHdl);
        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<short> > >())    return new ShmFw::HandlerParameter<short>( name, shmHdl);
        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<int> > >())    return new ShmFw::HandlerParameter<int>( name, shmHdl);
        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<long> > >())    return new ShmFw::HandlerParameter<long>( name, shmHdl);
        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<long long> > >())    return new ShmFw::HandlerParameter<long long>( name, shmHdl);

        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<unsigned char> > >())    return new ShmFw::HandlerParameter<unsigned char>( name, shmHdl);
        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<unsigned short> > >())    return new ShmFw::HandlerParameter<unsigned short>( name, shmHdl);
        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<unsigned int> > >())    return new ShmFw::HandlerParameter<unsigned int>( name, shmHdl);
        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<unsigned long> > >())    return new ShmFw::HandlerParameter<unsigned long>( name, shmHdl);
        if(shmHeader.isType<ShmFw::Var< ShmFw::ParameterEntry<unsigned long long> > >())    return new ShmFw::HandlerParameter<unsigned long long>( name, shmHdl);

        std::cerr << "unkonw type\n";
        throw 0;
    }

};



#endif //SHARED_MEM_OBJECT_HANDLER_PARAMETERENTRY




