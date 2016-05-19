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

#ifndef SHARED_MEM_MODULE_OSZI_H
#define SHARED_MEM_MODULE_OSZI_H

#include <iostream>
#include <fstream>
#include <opencv/cv.h>
#include <shmfw/vector.h>
#include <shmfw/serialization/vector.h>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/thread.hpp>

namespace ShmFw {
  /**
   * Class plots a ShmFw::Vector<double> entries into a window like an oscilloscope.
   **/
class Oszi {
public:
    static const int ROWS = 4;
    static const int COLS = 4;
    static const unsigned char COLORS[11][3];
    static const int WHITE = 0;
    static const int BLACK = 1;
    static const int BLUE = 2;
    static const int GREEN = 3;
    static const int RED = 4;
    static const int BLUE_LIGHT = 5;
    static const int GREEN_LIGHT = 6;
    static const int RED_LIGHT = 7;
    static const int YELLOW = 8;
    static const int MAGENTA = 9;
    static const int CYAN = 10;

    /** Constructor
     * @param name name of the variable
     * @param shmHdl pointer to the shared memory segment
     * @pre the ShmPtr poitner must be created first
     * @see ShmFw::createSegment
     **/
    Oszi ( const std::string &name, HandlerPtr &shmHdl );
    ~Oszi();
    class ChannelInfo {
    public:
        ChannelInfo();
        void init ( const std::string &_name, double _scale = 1., double _offset = 0, int _colorId = GREEN, double _lineWidth = 1., double _lineType = 1. );
        bool enabled;
        std::string name;
        double scale;
        double offset;
        double lineWidth;
        double lineType;
        unsigned char color[3];
    protected:
        friend class boost::serialization::access;
        /** Boost serialization function **/
        template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
            using boost::serialization::make_nvp;
            ar & make_nvp ( "enabled", enabled );
            ar & make_nvp ( "name", name );
            ar & make_nvp ( "scale", scale );
            ar & make_nvp ( "offset", offset );
            ar & make_nvp ( "lineWidth", lineWidth );
            ar & make_nvp ( "lineType", lineType );
            ar & make_nvp ( "color", color );
        }
    };
    /** Write into a file **/
    void write ( const std::string &filename ) {
        std::ofstream ofs ( filename.c_str() );
        assert ( ofs.good() );
        boost::archive::xml_oarchive xml ( ofs );
        xml << boost::serialization::make_nvp ( "Oszi", *this );
    }
    /** Read from a file **/
    void read ( const std::string &filename ) {
        std::ifstream ifs ( filename.c_str() );
        assert ( ifs.good() );
        boost::archive::xml_iarchive xml ( ifs );
        xml >> boost::serialization::make_nvp ( "Oszi", *this );
    }
    const cv::Mat &view() const {
        return view_;
    }
    void start();
    void update();
    void show ( int ms );
    void clear_history();
    void update_history();
    void drawGrid();
    void captureImages ( bool flag );
    void drawInfo ( int channel );
    cv::Point signal_to_screen ( unsigned int channel, unsigned int idx );

    const cv::Rect& resetButton() const;
    const cv::Rect& closeButton() const;
    bool close() {
        return close_;
    };
    void close(bool value) {
        close_ = value;
    };

private:
    bool close_;
    std::string capture_folder;
    int capture_counter;
    cv::Mat view_;
    ShmFw::Vector<double> channelsShm_;
    std::vector<double> channels_;
    cv::Rect size_screen_;
    std::vector< std::deque<double> > channel_histories_;
    std::vector< ChannelInfo > channel_infos_;
    std::deque<boost::posix_time::ptime> timestamps_;
    double resolution_time_;
    int trigger_channel_;
    int border_;
    double textHeight_;
    double titleHeight_;
    double textChannelHeight_;
    double trigger_value_;
    boost::thread *pListener;
    boost::interprocess::interprocess_mutex mutex;
    cv::Rect resetButton_;
    cv::Rect closeButton_;

    friend class boost::serialization::access;
    /** Boost serialization function **/
    template<class archive>  void serialize ( archive &ar, const unsigned int version ) {
        using boost::serialization::make_nvp;
        if ( archive::is_saving::value ) {
            ar & make_nvp ( "height", view_.rows );
            ar & make_nvp ( "width", view_.cols );
        }
        if ( archive::is_loading::value ) {
            int rows, cols;
            ar & make_nvp ( "height", rows );
            ar & make_nvp ( "width", cols );
            view_ = cv::Mat ( rows, cols, CV_8UC3 );
        }
        ar & make_nvp ( "border", border_ );
        ar & make_nvp ( "textHeight", textHeight_ );
        ar & make_nvp ( "titleHeight", titleHeight_ );
        ar & make_nvp ( "textChannelHeight", textChannelHeight_ );
        ar & make_nvp ( "trigger_channel", trigger_channel_ );
        ar & make_nvp ( "trigger_value", trigger_value_ );
        ar & make_nvp ( "size_screen.x", size_screen_.x );
        ar & make_nvp ( "size_screen.y", size_screen_.y );
        ar & make_nvp ( "size_screen.width", size_screen_.width );
        ar & make_nvp ( "size_screen.width", size_screen_.height );
        ar & make_nvp ( "resolution_time", resolution_time_ );
        ar & make_nvp ( "channel_infos", channel_infos_ );
    }
};
};
#endif //SHARED_MEM_MODULE_OSZI_H

