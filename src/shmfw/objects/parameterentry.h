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

#ifndef SHARED_MEM_OBJECT_PARAMETER_ENTRY
#define SHARED_MEM_OBJECT_PARAMETER_ENTRY

#include <iostream>
#include <iomanip>
#include <string.h>
#include <typeinfo>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>

namespace ShmFw {

class HandlerParameterBase;
template<class T> class HandlerParameter;

template <class T = int>
class ParameterEntry {
    friend class HandlerParameter<T>;
protected:
    T value_;
    T min_;
    T max_;
    bool enable_;
    uint8_t type_;
    T step_size_;
public:
    static const uint8_t TYPE_VALUE = 0;
    static const uint8_t TYPE_TRACKBAR = 1;
    static const uint8_t TYPE_CHECKBOX = 2;
    static const uint8_t TYPE_TEXT = 3;
    static const int EXEPTION_TYPE_ASSIGNMENT = 10;
    static const int EXEPTION_TYPE_COMPARE = 20;
    ParameterEntry() {
    }
    ParameterEntry(const T &_value)
        : value_(_value)
        , min_(_value)
        , max_(_value)
        , enable_(true)
        , type_(TYPE_VALUE) {
    }
    ParameterEntry(const ParameterEntry<T> &p)
        : value_(p.value_), min_(p.min_), max_(p.max_), enable_(p.enable_), type_(TYPE_VALUE), step_size_(p.step_size_) {
    }
    ParameterEntry(const T &_value, const T &_min, const T &_max, const bool &_enable = true)
        : value_(_value), min_(_min), max_(_max), enable_(_enable), type_(TYPE_VALUE), step_size_() {
    }
    ParameterEntry(const T &_value, const T &_min, const T &_max, const T& _step_size, const bool &_enable = true)
        : value_(_value), min_(_min), max_(_max), enable_(_enable), type_(TYPE_VALUE), step_size_(_step_size) {
    }

    /** sets enable
     * @param new value
     * @return enable
     **/
    bool enable( bool _enable ) {
        return enable_ = _enable;
    }
    /** returns enable value
     * @return enable
     **/
    bool enable() const {
        return enable_;
    }
    /** casts the shared variable
     * @return true on equal
     **/
    template <class T2>
    ParameterEntry<T2> *cast () {
        return (ParameterEntry<T2> *) this;
    }
    /** Assignment operator
     * @return this
     **/
    ParameterEntry<T> operator = ( const ParameterEntry<T>& p ) {
        value_ = p.value_, min_ = p.min_, max_ = p.max_, step_size_ = p.step_size_, enable_ = p.enable_;
        return *this;
    }
    /** Assignment operator
     * @return bool
     **/
    T operator = ( const T & _value ) {
        value_ = _value;
        return value_;
    }
    /** Assignment operator
     * @return bool
     **/
    bool operator == ( const ParameterEntry<T>& p ) const {
        return (value_ == p.value_) && (min_ == p.min_) && (max_ == p.max_) &&
               (step_size_ == p.step_size_) && (enable_ == p.enable_);
    }
    /** Assignment operator
     * @return bool
     **/
    bool operator == ( const T& _value ) const {
        return value_ == _value;
    }
    /** Assignment operator
     * @return bool
     **/
    bool operator < ( const ParameterEntry<T>& p ) const {
        return value_ < p.value_;
    }
    /** Assignment operator
     * @return bool
     **/
    bool operator < ( const T& _value ) const {
        return value_ < _value;
    }
    /** Assignment operator
     * @return bool
     **/
    bool operator <= ( const ParameterEntry<T>& p ) const {
        return value_ <= p.value_;
    }
    /** Assignment operator
     * @return bool
     **/
    bool operator <= ( const T& _value ) const {
        return value_ <= _value;
    }
    /** Assignment operator
     * @return bool
     **/
    bool operator > ( const ParameterEntry<T>& p ) const {
        return value_ > p.value_;
    }
    /** Assignment operator
     * @return bool
     **/
    bool operator > ( const T& _value ) const {
        return value_ > _value;
    }
    /** Assignment operator
     * @return bool
     **/
    bool operator >= ( const ParameterEntry<T>& p ) const {
        return value_ >= p.value_;
    }
    /** Assignment operator
     * @return bool
     **/
    bool operator >= ( const T& _value ) const {
        return value_ >= _value;
    }
    /** sets value
     * @return value
     **/
    T value( const T& _value ) {
        return value_ = _value;
    }
    /** returns value
     * @return value
     **/
    T value() const {
        return value_;
    }
    /** sets value
     * @return minimum
     **/
    T min( const T& _min ) {
        return min_ = _min;
    }
    /** returns minimum value
     * @return minimum
     **/
    T min() const {
        return min_;
    }
    /** sets maximum
     * @return maximum
     **/
    T max( const T& _max ) {
        return max_ = _max;
    }
    /** returns maximum value
     * @return maximum
     **/
    T max() const {
        return max_;
    }
    /** sets step_size
     * @return step_size
     **/
    T step_size(const T &_step_size ) {
        return step_size_ = _step_size;
    }
    /** returns step_size value
     * @return step_size
     **/
    T step_size() const {
        return step_size_;
    }
    /** checks if the parameter value is between min and max and enable
     * @return enable
     **/
    bool valid() const {
        return (value_ >= min_) && (value_ <= max_);
    }
    /** increases value one step
     * @return value
     **/
    void increase() {
        T v = value_ + step_size_;
        if(v <= max_){
         value_ = v;
        }
    }   
    /** decrease value one step
     * @return value
     **/
    void decrease() {
        T v = value_ - step_size_;
        if(v >= min_){
         value_ = v;
        }
    }
    /** output opeartor
     * @return stream
     **/
    friend std::ostream &operator << ( std::ostream &os, const ParameterEntry<T> &o ) {
        os << "["  << std::setw ( 3 ) << o.value_ << ", "  << std::setw ( 3 ) << o.min_ << ", " << std::setw ( 3 ) << o.max_;
        os << ( o.enable_?", enable":", disabled" ) << ", " << std::setw ( 3 ) << o.step_size_ << "]" ;
        return os;
    };
    
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "value", value_ );
        ar & make_nvp ( "min", min_ );
        ar & make_nvp ( "max", max_ );
        ar & make_nvp ( "enable", enable_ );
        ar & make_nvp ( "type", type_ );
        ar & make_nvp ( "step_size", step_size_ );
    }
};



};



#endif //SHARED_MEM_OBJECT_PARAMETER_ENTRY




