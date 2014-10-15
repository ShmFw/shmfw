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

#ifndef EIGEN_BOOST_SERIALIZATION
#define EIGEN_BOOST_SERIALIZATION
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/nvp.hpp>

namespace boost {
namespace serialization {
template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void save ( Archive & ar, const Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version ) {
    int rows=m.rows(),cols=m.cols();
    ar & boost::serialization::make_nvp ( "rows", rows);
    ar & boost::serialization::make_nvp ( "cols", cols);
    ar &  boost::serialization::make_nvp ( "data", boost::serialization::make_array ( m.data(), rows*cols ));
}
template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void load ( Archive & ar, Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version ) {
    int rows,cols;
    ar & boost::serialization::make_nvp ( "rows", rows);
    ar & boost::serialization::make_nvp ( "cols", cols);
    m.resize ( rows,cols );
    ar &  boost::serialization::make_nvp ( "data", boost::serialization::make_array ( m.data(), rows*cols ));
}

template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void serialize ( Archive & ar, Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version ) {
    split_free ( ar,m,version );
}


template <class Archive, typename _Scalar>
void save ( Archive & ar, const Eigen::Triplet<_Scalar> & m, const unsigned int version ) {
    ar & boost::serialization::make_nvp ( "rows", m.row());
    ar & boost::serialization::make_nvp ( "cols", m.col());
    ar & boost::serialization::make_nvp ( "value", m.value());
}
template <class Archive, typename _Scalar>
void load ( Archive & ar, Eigen::Triplet<_Scalar> & m, const unsigned int version ) {
    int row,col;
    _Scalar value;
    ar & boost::serialization::make_nvp ( "rows", row);
    ar & boost::serialization::make_nvp ( "cols", col);
    ar & boost::serialization::make_nvp ( "value", value);
    m = Eigen::Triplet<_Scalar> ( row,col,value );
}

template <class Archive, typename _Scalar>
void serialize ( Archive & ar, Eigen::Triplet<_Scalar> & m, const unsigned int version ) {
    split_free ( ar,m,version );
}


template <class Archive, typename _Scalar, int _Options,typename _Index>
void save ( Archive & ar, const Eigen::SparseMatrix<_Scalar,_Options,_Index> & m, const unsigned int version ) {
    int innerSize=m.innerSize();
    int outerSize=m.outerSize();
    typedef typename Eigen::Triplet<_Scalar> Triplet;
    std::vector<Triplet> triplets;
    for ( int i=0; i < outerSize; ++i ) {
        for ( typename Eigen::SparseMatrix<_Scalar,_Options,_Index>::InnerIterator it ( m,i ); it; ++it ) {
            triplets.push_back ( Triplet ( it.row(), it.col(), it.value() ) );
        }
    }
    ar & boost::serialization::make_nvp ( "innerSize", innerSize);
    ar & boost::serialization::make_nvp ( "outerSize", outerSize);
    ar & boost::serialization::make_nvp ( "triplets", triplets);
}
template <class Archive, typename _Scalar, int _Options, typename _Index>
void load ( Archive & ar, Eigen::SparseMatrix<_Scalar,_Options,_Index> & m, const unsigned int version ) {
    int innerSize;
    int outerSize;
    ar & boost::serialization::make_nvp ( "innerSize", innerSize);
    ar & boost::serialization::make_nvp ( "outerSize", outerSize);
    int rows = m.IsRowMajor?outerSize:innerSize;
    int cols = m.IsRowMajor?innerSize:outerSize;
    m.resize ( rows,cols );
    typedef typename Eigen::Triplet<_Scalar> Triplet;
    std::vector<Triplet> triplets;
    ar & boost::serialization::make_nvp ( "triplets", triplets);
    m.setFromTriplets ( triplets.begin(), triplets.end() );

}
template <class Archive, typename _Scalar, int _Options, typename _Index>
void serialize ( Archive & ar, Eigen::SparseMatrix<_Scalar,_Options,_Index> & m, const unsigned int version ) {
    split_free ( ar,m,version );
}

}
}
#endif
