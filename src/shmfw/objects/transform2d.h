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
#ifndef SHARED_MEM_OBJECTS_TRANSFORM2D_H
#define SHARED_MEM_OBJECTS_TRANSFORM2D_H


#include <shmfw/objects/point2d.h>
#include <shmfw/objects/pose2d.h>
#include <shmfw/objects/matrix3x3.h>
#include <math.h>

namespace ShmFw {

class Transform2D : public ShmFw::BaseObject {
private:
    ShmFw::Matrix3x3<double> M; // Transformation Matrix
public:
    /// construtor --> this one does not initialize the tf
    Transform2D();
    /// copy construtor --> this one does not initialize the tf
    Transform2D ( const Transform2D &o );
    /**
     * construtor
     * @param dx translation x
     * @param dy translation y
     * @param da rotation
     **/
    Transform2D ( const double &dx, const double &dy, const double &da );

    /**
     * construtor
     * @param o Pose2D
     **/
    Transform2D ( const Pose2D &o );
    /**
     * construtor crates a transformation from a to b
     * @param a  Pose2D
     * @param b  Pose2D
     **/
    Transform2D ( const Pose2D &a, const Pose2D &b );
    
    /// construtor
    Transform2D ( const Matrix3x3<double> &o );

    /** returns a string presenting the transformation
     * @param str format string
     * @see getToString()
     * @return string
     **/
    std::string getToStringFormat ( const std::string &format ) const;

    /** returns a string presenting the transformation with x, y, orientation
     * @return string
     **/
    std::string getToString() const;

    /** reads values from a string
     * @param str
     * @return false on error
     **/
    bool setFromString ( const std::string &str ) ;

    /** output operator
     **/
    friend std::ostream& operator<< ( std::ostream &output, const Transform2D &o ) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>> ( std::istream &input, Transform2D &o ) {
        std::string str;
        getline ( input, str );
        o.setFromString ( str );
        return input;
    }
    /** @return true on equal
     **/
    bool operator == ( const Transform2D& o ) const;
    
    /** @return invert transforamtion
     **/
    Transform2D invert (); 

    /**
     * Transform
     * @param o to transform
     * @return point transformed
     **/
    Point2D operator * ( const Point2D& o ) const;
    /**
     * Invert transform
     * @param o to transform
     * @return point transformed
     **/
    Point2D operator / ( const Point2D& o ) const;

    /**
     * combine Transforms
     * @param o transforamtion to apply
     * @return this transformed
     **/
    const Transform2D &operator *= ( const Transform2D& o );
    
    /**
     * combine Transforms
     * @param o transforamtion to apply
     * @return new transform 
     **/
    Transform2D operator * ( const Transform2D& o ) const;
    
    /**
     * Transform
     * @param o to transform
     * @return Pose2D transformed
     **/
    Pose2D operator * ( const Pose2D& o ) const;

    /** compares with within a tolerance
     * @param o
     * @param tolerance
     **/
    bool equal ( const Transform2D& o, double tolerance = 0.0001 ) const;

    /** translational x component
     * @return M(0,2) --> x component
     **/
    const double &x () const;

    /** translational y component
     * @return M(1,2) --> y component
     **/
    const double &y () const;

    /** rotation component
     * @return acos(M(0,0)
     **/
    double orientation () const;

    /** Computes a pose element
     * @param des data source
     * @return pose reference to the argument
     **/
    ShmFw::Pose2D &getPose ( ShmFw::Pose2D &des ) const;

    /** Computes a pose element
     * @return pose
     **/
    Pose2D getPose () const;
  
    /** Computes a pose element
     * @param src data source
     * @return reference to this
     **/
    ShmFw::Transform2D &setTf ( const ShmFw::Pose2D &src );

    /** sets the transformation components
     * @param dx translation x
     * @param dy translation y
     * @param da rotation
     * @return reference to this
     **/
    ShmFw::Transform2D &setTf ( const double &dx, const double &dy, const double &da );

    /** Computes a transformation from a to b
     * @param a  Pose2D
     * @param b  Pose2D
     * @return reference to this
     **/
    ShmFw::Transform2D &setTf ( const ShmFw::Pose2D &a,  const ShmFw::Pose2D &b );
    
    /** sets the identity -> no transformation
     **/
    void identity ();

    /** @returns the identity
     **/
    static const ShmFw::Transform2D IDENTITY ();

protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "M", M );
    }
};


};
#endif //SHARED_MEM_OBJECTS_TRANSFORM2D_H

