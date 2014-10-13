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
#ifndef SHARED_MEM_OBJECTS_RGBA_H
#define SHARED_MEM_OBJECTS_RGBA_H

#include <shmfw/objects/rgb.h>

namespace ShmFw {

class RGBA {
public:
    float r, g, b, a;
    RGBA(): r(.0), g(.0), b(.0), a(.0) {};
    RGBA(const RGBA &p): r(p.r), g(p.g), b(p.b), a(p.a) {};
    RGBA(float r, float g, float b, float a): r(r), g(g), b(b), a(a) {};
    RGBA(const RGB &c, float A) {
      setColour(c, A);
    };
    std::string getToString() const{
      char buf[0xFF];
      sprintf(buf, "[ %f, %f, %f, %f]", r, g, b, a); 
      return std::string(buf);
    }
    bool setFromString(const std::string &str) {
      int start = str.find("[");
      int end = str.find_last_of("]");
      std::string data = str.substr(start+1, end-1); 
      boost::erase_all(data, " ");
      if(sscanf(data.c_str(), "%f,%f,%f,%f", &r, &g, &b, &a) == EOF) return false; 
      return true;
    }
    friend std::ostream& operator<< (std::ostream &output, const RGBA &o) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>>(std::istream &input, RGBA &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
     /**
      * sets 0 0 0 1
     **/
    void zero(){
      r = 0, g = 0, b = 0, a = 1.;
    }
    void setColour(const RGB &c, float A = 1){
      r = ((float) c.r) / 255.0, g = ((float) c.g) / 255.0, b = ((float) c.b) / 255.0, a = A;
    }
    void setColour(float _r, float _g, float _b, float _a = 1.){
      r = _r, g = _g, b = _b, a = _a;
    }
    bool operator == ( const RGBA& o ) const {
        return r == o.r && g == o.g && b == o.b && a == o.a;
    } 
    template<typename T>
    void copyTo ( T& des ) const {
        des.r = r, des.g = g, des.b = b, des.a = a;
    }
    template<typename T>
    RGBA& copyFrom ( const T& src ) {
        r = src.r, g = src.g, b = src.b, a = src.a;
        return *this;
    }
    /// http://www.rapidtables.com/web/color/RGB_Color.htm
    static const RGBA basic_colors(int i){
      switch(i){
	case 0: return black();
	case 1: return white();
	case 2: return red();
	case 3: return lime();
	case 4: return blue();
	case 5: return yellow();
	case 6: return cyan();
	case 7: return magenta();
	case 8: return sivler();
	case 9: return gray();
	case 10: return maroon();
	case 11: return olive();
	case 12: return green();
	case 13: return purple();
	case 14: return teal();
	case 15: return navy();
      }
      return black();
    }
    static const RGBA black(){
      return RGBA(0,0,0,1);
    }
    static const RGBA white(){
      return RGBA(1,1,1,1);
    }
    static const RGBA red(){
      return RGBA(1,0,0,1);
    }
    static const RGBA lime(){
      return RGBA(0,1,0,1);
    }
    static const RGBA blue(){
      return RGBA(0,0,1,1);
    }
    static const RGBA yellow(){
      return RGBA(1,1,0,1);
    }
    static const RGBA cyan(){
      return RGBA(0,1,1,1);
    }
    static const RGBA magenta(){
      return RGBA(1,0,1,1);
    }
    static const RGBA sivler(){
      return RGBA(0.75,0.75,0.75,1);
    }
    static const RGBA gray(){
      return RGBA(0.5,0.5,0.5,1);
    }
    static const RGBA maroon(){
      return RGBA(0.5,0,0,1);
    }
    static const RGBA olive(){
      return RGBA(0.5,0.5,0,1);
    }
    static const RGBA green(){
      return RGBA(0,0.5,0,1);
    }
    static const RGBA purple(){
      return RGBA(0.5,0,0.5,1);
    }
    static const RGBA teal(){
      return RGBA(0,0.5,0.5,1);
    }
    static const RGBA navy(){
      return RGBA(0,0,0.5,1);
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "r", r );
        ar & make_nvp ( "g", g );
        ar & make_nvp ( "b", b );
        ar & make_nvp ( "a", a );
    }
    
};
};
#endif //SHARED_MEM_OBJECTS_RGBA_H

