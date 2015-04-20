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
#ifndef SHARED_MEM_OBJECTS_TRANSFORM_H
#define SHARED_MEM_OBJECTS_TRANSFORM_H

#include <shmfw/objects/vector3.h>
#include <shmfw/objects/matrix3x3.h>
#include <boost/graph/graph_concepts.hpp>

namespace ShmFw {
class Quaternion;
class Transform {
public:
   
    Matrix3x3<double> basis;   /// Storage for the rotation
    Vector3<double>   origin;  /// Storage for the translation
    Transform(); /// Constructor
    Transform( const Transform &o);    /// Copy constructor
    /** Constructor
     * @param _basis rotation
     * @param _origin translation
     **/
    Transform(const Matrix3x3<double> &_basis, const Vector3<double> _origin);  
    /** Constructor
     * @param _basis rotation
     * @param _origin translation
     **/
    Transform(const Quaternion &_basis, const Vector3<double> _origin);  
    /** returns a string reprecenting the transform
     * @return string
     * @see setFromString
     **/
    std::string getToString() const;  
    /** converts a string into atransform
     * @param str string reprecenting the transform
     * @return string
     * @see getToString
     **/
    bool setFromString ( const std::string &str );
    /** compares 
     * @return true on equal
     **/
    bool operator == ( const Transform& o ) const {
        return basis == o.basis && origin == o.origin;
    }
    /** compares with within a expsilon range
     * @param o
     * @param tolerance
     **/
    bool equal ( const Transform& o, double tolerance = 0.0001 ) const {
        return basis.equal ( o.basis, tolerance ) && origin.equal ( o.origin, tolerance );
    }
    
    /** Copies data to a ROS element
     * @param des data target
     **/
    template<typename T>
    void copyToROS ( T& des ) const {
        basis.copyToROS ( des.getBasis() );
        origin.copyToROS ( des.getOrigin() );
    }
    /** Copies data from a ROS element
     * @param des data target
     **/
    template<typename T>
    Transform& copyFromROS ( T& src ) {
        basis.copyFromROS ( src.getBasis() );
        origin.copyFromROS ( src.getOrigin() );
        return *this;
    }
    friend std::ostream& operator<< ( std::ostream &output, const Transform &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, Transform &o ) {
        std::string str;
        getline ( input, str );
        o.setFromString ( str );
        return input;
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "basis", basis );
        ar & make_nvp ( "origin", origin );
    }
};

};
#endif //SHARED_MEM_OBJECTS_TRANSFORM_H

