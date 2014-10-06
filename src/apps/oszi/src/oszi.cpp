/***************************************************************************
 *   Copyright (C) 2012 by Markus Bader                                    *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#include "shmfw/oszi.h"
#include "opencv/highgui.h"
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem/operations.hpp>

using namespace ShmFw;
static void onMouse ( int event, int x, int y, int, void* p ) {
    if ( event == CV_EVENT_FLAG_LBUTTON ) {
	  
        Oszi &oszi = * ( Oszi * ) p;
		if(oszi.resetButton().contains(cv::Point(x,y))){
		  oszi.clear_history();
		}
		if(oszi.closeButton().contains(cv::Point(x,y))){
		  oszi.close(true);
		}
    }
}
void callback ( ShmFw::Vector<double> *channels,  std::vector< std::deque<double> > *histories ) {
    for ( unsigned int i = 0; true; i++ ) {
        if ( channels->timed_wait ( 1000 ) == false ) {
            std::cout << "timeout" << std::endl;
        } else {
        }
    }
};

const unsigned char Oszi::COLORS[11][3] = { {0xFF, 0xFF, 0xFF}, {0x00, 0x00, 0x00},
    {0xFF, 0x00, 0x00}, {0x00, 0xFF, 0x00}, {0x00, 0x00, 0xFF},
    {0xb8, 0x00, 0x00}, {0x00, 0xb8, 0x00}, {0x00, 0x00, 0xb8},
    {0x00, 0xFF, 0xFF}, {0xFF, 0x00, 0xFF}, {0xFF, 0xFF, 0x00}
};

Oszi::Oszi ( const std::string &name, HandlerPtr &shmHdl )
    : channelsShm_ ( name,shmHdl ) {
	close_ = true;
    capture_counter = 0;
    view_ = cv::Mat ( 400, 800, CV_8UC3 );
    size_screen_ = cv::Rect ( 5,5, view_.rows - 10, view_.rows - 10 );
    cv::namedWindow ( channelsShm_.name(), CV_WINDOW_AUTOSIZE );
    resolution_time_ = 0.01;
    channel_infos_.resize ( 10 );
    trigger_channel_ = 0;
    trigger_value_ = 0.0;
    border_ = 10;
    textHeight_ = 0.3;
    titleHeight_ = 0.5;
    textChannelHeight_ = 0.8;
    pListener = NULL;
    clear_history();
    mutex.unlock();
    channelsShm_.unlock();
};

Oszi::~Oszi ( ) {
    cv::destroyWindow ( channelsShm_.name() );
    pListener->interrupt();
};


Oszi::ChannelInfo::ChannelInfo() {
    init ( "na" );
}

void Oszi::ChannelInfo::init ( const std::string &_name, double _scale, double _offset, int _colorId, double _lineWidth, double _lineType ) {
    name = _name;
    enabled = true;
    scale = _scale;
    offset = _offset;
    lineWidth = _lineWidth;
    lineType = _lineType;
    for ( int i = 0; i < 3; i++ ) color[i] = ShmFw::Oszi::COLORS[_colorId][i];
}


void Oszi::clear_history() {
    channel_histories_.clear();
    timestamps_.clear();
}
void Oszi::update_history() {
    for ( unsigned int c = 0; c < channel_histories_.size(); c++ ) {
        channel_histories_[c].pop_front();
    }
    timestamps_.pop_front();
}

void Oszi::update() {
    for ( unsigned int waitCounter = 0; true; waitCounter++ ) {
        while ( channelsShm_.timed_wait ( 1000 ) == false ) {
            std::cout << std::setw ( 4 ) << waitCounter << ": waited 1000 ms" << std::endl;
        }
        bool updateHistory = true;
        boost::posix_time::ptime t = channelsShm_.get ( channels_ );
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lockLocal ( mutex );
        if ( ( channel_histories_.size() != channels_.size() ) ) {
            clear_history();
        }
        if ( timestamps_.size() > 0 ) {
            double d = ( ( double ) ( t -  timestamps_[0] ).total_microseconds() ) / 1000000.0;
            double x = d / resolution_time_;
            if ( x > size_screen_.width ) {
                if ( trigger_channel_ >= 0 ) {
                    if ( channels_[trigger_channel_] > trigger_value_ ) {
                        clear_history();
                    } else {
                        updateHistory = false;
                    }
                } else {
                    update_history();
                }
            }
        }
        if ( updateHistory ) {
            channel_histories_.resize ( channels_.size() );
            timestamps_.push_back ( t );
            for ( unsigned int i = 0; i < channels_.size(); i++ ) {
                channel_histories_[i].push_back ( channels_[i] );
            }
        }
    }
}

void Oszi::drawGrid () {
    view_.setTo ( cv::Scalar ( 0xF0, 0xF0, 0xF0 ) );
    cv::Mat screen ( view_, size_screen_ );
    screen.setTo ( cv::Scalar ( 0xFF, 0xFF, 0xFF ) );


    /// Center Cross
    cv::line ( screen, cv::Point ( size_screen_.width/2, 0 ), cv::Point ( size_screen_.width/2, size_screen_.height ), cv::Scalar ( 128, 128, 128 ), 1 );
    cv::line ( screen, cv::Point ( 0, size_screen_.height/2 ), cv::Point ( size_screen_.width, size_screen_.height/2 ), cv::Scalar ( 128, 128, 128 ), 1 );

    /// Vertical lines
    for ( int i = 0; i < ROWS; i++ ) {
        cv::Point pt0 ( size_screen_.width/ROWS*i, 0 ), pt1 ( size_screen_.width/ROWS*i, size_screen_.height );
        cv::line ( screen, pt0, pt1, cv::Scalar ( 200, 200, 200 ), 1 );
        char pInfo[0xFF];
        sprintf ( pInfo, "%4.2f sec", ( pt1.x - size_screen_.width/2 ) * resolution_time_ );
        cv::putText ( view_, pInfo, pt1, cv::FONT_HERSHEY_SIMPLEX, textHeight_, cv::Scalar ( 0x00,0x00,0x00 ), 1, CV_AA );
    }

    /// Horizontal lines
    for ( int i = 0; i < COLS; i++ ) {
        cv::Point pt0 ( 0, size_screen_.height/ROWS*i ), pt1 ( size_screen_.width, size_screen_.height/ROWS*i );
        cv::line ( screen, pt0, pt1, cv::Scalar ( 200, 200, 200 ), 1 );
    }

    /// Box
    cv::rectangle ( view_, size_screen_, cv::Scalar ( 0, 0, 0 ), 1 );  // Box;

    /// Reset
    resetButton_ = cv::Rect ( view_.cols-50, view_.rows-25, 40, 15);
    cv::rectangle ( view_, resetButton_, cv::Scalar ( 0, 0, 0 ), 1 );  // Box;
    cv::putText ( view_, "Reset", cv::Point(resetButton_.x+4, resetButton_.y+resetButton_.height-4), cv::FONT_HERSHEY_SIMPLEX, textHeight_, cv::Scalar ( 0x00,0x00,0x00 ), 1, CV_AA );
  
	/// Close
    closeButton_ = cv::Rect ( resetButton_.x-50, view_.rows-25, 40, 15);
    cv::rectangle ( view_, closeButton_, cv::Scalar ( 0, 0, 0 ), 1 );  // Box;
    cv::putText ( view_, "Close", cv::Point(closeButton_.x+4, closeButton_.y+closeButton_.height-4), cv::FONT_HERSHEY_SIMPLEX, textHeight_, cv::Scalar ( 0x00,0x00,0x00 ), 1, CV_AA );

}

const cv::Rect& Oszi::resetButton() const {
    return resetButton_;
};

const cv::Rect& Oszi::closeButton() const {
    return closeButton_;
};

cv::Point Oszi::signal_to_screen ( unsigned int channel, unsigned int idx ) {
    cv::Point pt ( size_screen_.width/2,size_screen_.height/2 );
    if ( timestamps_.size() == 0 ) return pt;
    const ChannelInfo& channel_info = channel_infos_[channel];

    double rowHeight = size_screen_.width/ROWS;

    double signal_scale = ( rowHeight ) * ( channel_info.scale );
    double screen_offset = size_screen_.height/2;

    double d = ( ( double ) ( timestamps_[idx] -  timestamps_[0] ).total_microseconds() ) / 1000000.0;
    double x = d / resolution_time_;
    double &signal = channel_histories_[channel][idx];
    double y = - ( signal * signal_scale ) + screen_offset - ( channel_info.offset * rowHeight );
    pt.x = x;
    pt.y = y;
    return pt;
}

void Oszi::start() {
  close_ = false;
    if ( pListener == NULL ) {
        pListener =  new boost::thread ( boost::bind ( &Oszi::update, this ) );
    } else {
        std::cout << "oszi runns allready!" << std::endl;
    }

}

void Oszi::drawInfo ( int channel ) {
    const ChannelInfo &info = channel_infos_[channel];
    int textHeightPix = textHeight_ * 60.0;
    int titleHeightPix = titleHeight_ * 60.0;
    int textChannelHeightPix = textChannelHeight_ * 60.0;
    cv::Scalar color ( info.color[0], info.color[1], info.color[2] );
    cv::Point p0 ( size_screen_.x + size_screen_.width + border_, size_screen_.y + titleHeightPix + channel*textChannelHeightPix );
    cv::Point p1 = p0 + cv::Point ( border_, 0 );
    cv::Point p2 = p1 + cv::Point ( border_, 0 );
    cv::Point p3 = p2 + cv::Point ( 0, textHeightPix );
    cv::line ( view_, p0, p1, color, info.lineWidth, CV_AA );
    char pInfo[0xFF];
    sprintf ( pInfo, " %s: %4.2f", info.name.c_str(), channels_[channel] );
    cv::putText ( view_, pInfo, p2, cv::FONT_HERSHEY_SIMPLEX, titleHeight_, cv::Scalar ( 0x00,0x00,0x00 ), 1, CV_AA );
    sprintf ( pInfo, "scale:%4.2f, offset: %4.2f", info.scale, info.offset );
    cv::putText ( view_, pInfo, p3, cv::FONT_HERSHEY_SIMPLEX, textHeight_, cv::Scalar ( 0x00,0x00,0x00 ), 1, CV_AA );
}

void Oszi::show ( int ms ) {
    drawGrid();
    cv::Mat screen ( view_, size_screen_ );
    if ( timestamps_.size() == 0 ) return;
    else {
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lockLocal ( mutex );
        for ( unsigned int c = 0; c < channel_histories_.size(); c++ ) {
            const ChannelInfo &info = channel_infos_[c];
            if ( info.enabled == false ) {
                continue;
            }
            drawInfo ( c );
            std::deque<double> &channel = channel_histories_[c];
            cv::Point pt0 = signal_to_screen ( c,0 );
            for ( unsigned int idx = 1; idx < channel.size(); idx++ ) {
                cv::Point pt1 = signal_to_screen ( c,idx );
                cv::Scalar color ( info.color[0], info.color[1], info.color[2] );
                int lineWidth = info.lineWidth;
                int lineStyle = info.lineType;
                cv::line ( screen, pt0, pt1, color, lineWidth, lineStyle );
                pt0 = pt1;
            }
        }
    }
    cv::imshow ( channelsShm_.name(), view_ );
    cv::setMouseCallback ( channelsShm_.name(), onMouse, ( void* ) this );
    if ( !capture_folder.empty() ) {
        char filename[0xFF];
        sprintf ( filename, "/image-%04i.jpg", capture_counter );
        capture_counter++;
        std::string imagefile = capture_folder + std::string ( filename );
        cv::imwrite ( imagefile, view_ );
    }
    cv::waitKey ( ms );
}

void Oszi::captureImages ( bool flag ) {
    if ( flag ) {
        boost::posix_time::ptime t ( boost::posix_time::microsec_clock::local_time() );
        std::string subfolder ( boost::posix_time::to_iso_string ( t ).substr ( 0,15 ) );
        std::cout << subfolder << std::endl;
        capture_folder = "/tmp/oszi/";
        boost::filesystem::path dir_path ( capture_folder );
        if ( !boost::filesystem::exists ( dir_path ) || !boost::filesystem::is_directory ( dir_path ) ) {
            boost::filesystem::create_directory ( dir_path );
        }
        capture_folder += subfolder;
        dir_path = boost::filesystem::path ( capture_folder );
        if ( !boost::filesystem::exists ( dir_path ) || !boost::filesystem::is_directory ( dir_path ) ) {
            boost::filesystem::create_directory ( dir_path );
        }
    }
}
