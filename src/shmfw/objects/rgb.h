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
#ifndef SHARED_MEM_OBJECTS_RGB_H
#define SHARED_MEM_OBJECTS_RGB_H

#include <time.h>     
#include <opencv2/core/core.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/algorithm/string.hpp>

namespace ShmFw {

class RGB {
public:
    uint8_t r, g, b;
    RGB(): r(0), g(0), b(0) {};
    RGB(uint8_t r, uint8_t g, uint8_t b): r(r), g(g), b(b) {};
    RGB(const RGB &p): r(p.r), g(p.g), b(p.b) {};
    std::string getToString() const{
      char buf[0xFF];
      sprintf(buf, "[ %d, %d, %d]", r, g, b); 
      return std::string(buf);
    }
    bool setFromString(const std::string &str) {
      int start = str.find("[");
      int end = str.find_last_of("]");
      std::string data = str.substr(start+1, end-1); 
      boost::erase_all(data, " ");
      int R, G, B;
      if(sscanf(data.c_str(), "%d,%d,%d", &R, &G, &B) == EOF) return false; 
      r = R, g = G, b = B;
      return true;
    }
    friend std::ostream& operator<< (std::ostream &output, const RGB &o) {
        output << o.getToString();
        return output;
    }
    friend std::istream& operator>>(std::istream &input, RGB &o)
    {
        std::string str;
        getline (input, str);
	o.setFromString(str);
        return input;
    }
    void setColour(float _r, float _g, float _b){
      r = _r, g = _g, b = _b;
    }
    bool operator == ( const RGB& o ) const {
        return r == o.r && g == o.g && b == o.b;
    } 
    template<typename T>
    void copyTo ( T& des ) const {
        des.r = r, des.g = g, des.b = b;
    }
    template<typename T>
    RGB& copyFrom ( const T& src ) {
        r = src.r, g = src.g, b = src.b;
        return *this;
    }
    cv::Scalar cvScalar() const {
      return cv::Scalar(b,g,r);
    }
    static RGB BLACK() {
      return RGB(0,0,0);
    }
    static RGB SILVER() {
      return RGB(0xC0,0xC0,0xC0);
    }
    static RGB GRAY() {
      return RGB(0x80,0x80,0x80);
    }
    static RGB WHITE() {
      return RGB(0x00,0x00,0x00);
    }
    static RGB MAROON() {
      return RGB(0x80,0x00,0x00);
    }
    static RGB RED() {
      return RGB(0xFF,0x00,0x00);
    }
    static RGB PURPLE() {
      return RGB(0x80,0x00,0x80);
    }
    static RGB FUCHASIA() {
      return RGB(0xFF,0x00,0xFF);
    }
    static RGB GREEN() {
      return RGB(0x00,0x80,0x00);
    }
    static RGB LIME() {
      return RGB(0x00,0xFF,0x00);
    }
    static RGB OLIVE() {
      return RGB(0x80,0x80,0x00);
    }
    static RGB YELLOW() {
      return RGB(0xFF,0xFF,0x00);
    }
    static RGB NAVY() {
      return RGB(0x00,0x00,0x80);
    }
    static RGB BLUE() {
      return RGB(0x00,0x00,0xFF);
    }
    static RGB TEAL() {
      return RGB(0x00,0x80,0x80);
    }
    static RGB AQUA() {
      return RGB(0x00,0xFF,0xFF);
    }
    /**
     * Returns a random colour;
     * @return color based on 14 colours + back and white
     **/
    static RGB RAND() {
      srand (time(NULL));
      unsigned int i = rand() & 0xF;
      return COLOR(i);
    }
    /**
     * Returns a color based on the index;
     * @param i index
     * @return color based on 14 colours + back and white
     **/
    static RGB COLOR(unsigned int i) {
      i = i & 0xF;
      switch(i){
	case 0: return BLACK();
	case 1: return WHITE();
	case 2: return SILVER();
	case 3: return GRAY();
	case 4: return MAROON();
	case 5: return RED();
	case 6: return PURPLE();
	case 7: return FUCHASIA();
	case 8: return GREEN();
	case 9: return LIME();
	case 10: return OLIVE();
	case 11: return YELLOW();
	case 12: return NAVY();
	case 13: return BLUE();
	case 14: return AQUA();
	case 15: return TEAL();
      }
      return BLACK();
    }
    /**
     * Returns a color based on the index no black and white
     * @param i index
     * @return color based on 14 colours no black and white
     **/
    static RGB COLOR_NO_WB(unsigned int i) {
      i = i & 0xE;
      switch(i){
	case 0: return SILVER();
	case 1: return GRAY();
	case 2: return MAROON();
	case 3: return RED();
	case 4: return PURPLE();
	case 5: return FUCHASIA();
	case 6: return GREEN();
	case 7: return LIME();
	case 8: return OLIVE();
	case 9: return YELLOW();
	case 10: return NAVY();
	case 11: return BLUE();
	case 12: return AQUA();
	case 13: return TEAL();
      }
      return GRAY();
    }
protected:
    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        ar & make_nvp ( "r", r );
        ar & make_nvp ( "g", g );
        ar & make_nvp ( "b", b );
    }
};
};
#endif //SHARED_MEM_OBJECTS_RGB_H

