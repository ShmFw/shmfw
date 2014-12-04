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

#ifndef SHARED_MEM_OBJECTS_PROBABILTY_ENTRY_H
#define SHARED_MEM_OBJECTS_PROBABILTY_ENTRY_H

#include <math.h>

namespace shmfw {
class ProbabilityEntry {
public:
    typedef int8_t  cellType;
    typedef uint8_t cellTypeUnsigned;
    cellType entry;
    operator int() {
        return entry;    // cast operator
    }
    ProbabilityEntry &operator = ( const double& p );
    ProbabilityEntry &operator = ( const float& p );
    float getProbability() const;
    void setProbability ( const float& p );
    const cellType &getEntry () const;
    cellType &getEntry ();
    friend std::ostream& operator<< ( std::ostream &output, const ProbabilityEntry &o ) {
        output << o.getProbability();
        return output;
    }
    static float &l2p ( const cellType &l, float &p );
    static cellType &p2l ( const float &p, cellType &l );
private:
    static mrpt::slam::CLogOddsGridMapLUT<cellType>  m_logodd_lut;
};
};
#endif // SHARED_MEM_OBJECTS_PROBABILTY_ENTRY_H
