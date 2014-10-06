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

#include <shmfw/figure.h>

using namespace ShmFw;
cv::Scalar Figure::getColour(int i) {
    switch(i) {
    case WHITE:
        return cv::Scalar(0xFF, 0xFF, 0xFF);
    case BLACK:
        return cv::Scalar(0x00, 0x00, 0x00);
    case BLUE:
        return cv::Scalar(0xFF, 0x00, 0x00);
    case GREEN:
        return cv::Scalar(0x00, 0xFF, 0x00);
    case RED:
        return cv::Scalar(0x00, 0x00, 0xFF);
    case BLUE_LIGHT:
        return cv::Scalar(0xb8, 0x00, 0x00);
    case GREEN_LIGHT:
        return cv::Scalar(0x00, 0xb8, 0x00);
    case RED_LIGHT:
        return cv::Scalar(0x00, 0x00, 0xb8);
    case YELLOW:
        return cv::Scalar(0x00, 0xFF, 0xFF);
    case MAGENTA:
        return cv::Scalar(0xFF, 0x00, 0xFF);
    case CYAN:
        return cv::Scalar(0xFF, 0xFF, 0x00);
    };
    return cv::Scalar(0x00, 0x00, 0x00);
};
Figure::~Figure()
{
}

Figure::Figure () : loop_count_(0) {
}

Figure::Parameters &Figure::getParam() {
    return param_;
}
void  Figure::init() {
    view_ = cv::Mat ( param_.widthPix, param_.heightPix, CV_8UC3 );
}
void Figure::render() {
    view_.setTo ( param_.background );
    if(loop_count_ == 0) {
        cv::namedWindow ( param_.name, CV_WINDOW_AUTOSIZE );
    }
    draw();
    grid();
    cv::imshow ( param_.name, view_ );
    cv::waitKey ( param_.render_delay );
    loop_count_++;
}

Figure::Plot &Figure::getPlot(const std::string &name) {
    std::map<std::string, Plot>::iterator it = plots_.find("name");
    if(it == plots_.end()) {
        Plot p;
        p.name = name;
        plots_[name] = p;
    }
    return plots_[name];
}
void Figure::grid() {
    for (double y = -5; y <= 5; y+=1.0) {
        for (double x = -5; x <= 5; x+=1.0) {
            cv::circle(view_, homo_.meter2Pix(x,y), 1, cv::Scalar(0,0,0));
        }
    }
    cv::line(view_, homo_.meter2Pix(-5,0), homo_.meter2Pix(5,0), cv::Scalar(0,0,0));
    cv::line(view_, homo_.meter2Pix(0,-5), homo_.meter2Pix(0,5), cv::Scalar(0,0,0));

    cv::line(view_, homo_.meter2Pix(param_.x_max, param_.y_max), homo_.meter2Pix(param_.x_max, param_.y_min), cv::Scalar(0,0,0), 2);
    cv::line(view_, homo_.meter2Pix(param_.x_max, param_.y_min), homo_.meter2Pix(param_.x_min, param_.y_min), cv::Scalar(0,0,0), 2);
    cv::line(view_, homo_.meter2Pix(param_.x_min, param_.y_min), homo_.meter2Pix(param_.x_min, param_.y_max), cv::Scalar(0,0,0), 2);
    cv::line(view_, homo_.meter2Pix(param_.x_min, param_.y_max), homo_.meter2Pix(param_.x_max, param_.y_max), cv::Scalar(0,0,0), 2);

    //double widthMeter  = param_.x_max - param_.x_min;
    double heightMeter = param_.y_max -param_.y_min;

    cv::putText(view_, "X ", homo_.meter2Pix(param_.x_max, param_.y_min+heightMeter/2), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));
    cv::putText(view_, "Y ", homo_.meter2Pix(param_.x_min+heightMeter/2, param_.y_max), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,0));

    cv::line(view_, homo_.meter2Pix(0,-5), homo_.meter2Pix(0,5), cv::Scalar(0,0,0));
}
void Figure::draw() {

    if (param_.auto_size) {
        for (std::map<std::string,Plot>::iterator it=plots_.begin(); it!=plots_.end(); ++it) {
            Plot &plot = it->second;
            plot.findMaxima();
            if (it==plots_.begin()) {
                param_.x_max = plot.x_max, param_.x_min = plot.x_min;
                param_.y_max = plot.y_max, param_.y_min = plot.y_min;
            } else {
                if(param_.x_max < plot.x_max) param_.x_max = plot.x_max;
                if(param_.y_max < plot.y_max) param_.y_max = plot.y_max;
                if(param_.x_min > plot.x_min) param_.x_min = plot.x_min;
                if(param_.y_min > plot.y_min) param_.y_min = plot.y_min;
            }
        }
    }

    double widthMeter  = param_.x_max - param_.x_min;
    double heightMeter = param_.y_max -param_.y_min;
    double widthDrawPix = param_.widthPix*0.9;
    double heightDrawPix = param_.heightPix*0.9;
    double sx = widthDrawPix/widthMeter;
    double sy = heightDrawPix/heightMeter;
    double ox = - (param_.x_max * sx - widthDrawPix-(param_.widthPix-widthDrawPix)/2);
    double oy = - (param_.y_max * sy - heightDrawPix-(param_.heightPix-heightDrawPix)/2);

    homo_.init(sx, 0., ox, 0., sy, oy);

    for (std::map<std::string,Plot>::iterator it=plots_.begin(); it!=plots_.end(); ++it) {
        Plot &plot = it->second;
        cv::Point pt0, pt1, pt2, pt3, pt4;
        if(plot.type == LINE) {
            if(plot.points.size() > 0) {
                homo_.meter2Pix(plot.points[0], pt0);
            }
            for(unsigned i = 1; i  < plot.points.size(); i = i + plot.step_size) {
                homo_.meter2Pix(plot.points[i], pt1);
                cv::line(view_, pt0, pt1, plot.colour, plot.thickness, plot.lineType, plot.shift);
                pt0 = pt1;
            }
        } else {
            for(unsigned i = 0; i  < plot.points.size(); i = i + plot.step_size) {
                homo_.meter2Pix(plot.points[i], pt0);
                if( plot.type == DOT) {
                    setDot(pt0.x,pt0.y, plot.colour);
                }
                if( plot.type == DOT4) {
                    if((pt0.y+1 < view_.cols) && (pt0.y-1 >= 0) && (pt0.x+1 < view_.rows) && (pt0.x-1 >= 0)) {
                        setDot(pt0.x,pt0.y, plot.colour);
                        setDot(pt0.x+1,pt0.y, plot.colour);
                        setDot(pt0.x-1,pt0.y, plot.colour);
                        setDot(pt0.x,pt0.y+1, plot.colour);
                        setDot(pt0.x,pt0.y-1, plot.colour);
                    }
                }
                if( plot.type == CIRCLE) {
                    cv::circle(view_,pt0, plot.type_param,plot.colour,plot.thickness,plot.lineType,plot.shift);
                }
                if( plot.type == CROSS) {
                    pt1 = cv::Point(pt0.x+plot.type_param, pt0.y+plot.type_param);
                    pt2 = cv::Point(pt0.x-plot.type_param, pt0.y-plot.type_param);
                    pt3 = cv::Point(pt0.x-plot.type_param, pt0.y+plot.type_param);
                    pt4 = cv::Point(pt0.x+plot.type_param, pt0.y-plot.type_param);
                    cv::line(view_, pt1, pt2, plot.colour, plot.thickness, plot.lineType, plot.shift);
                    cv::line(view_, pt3, pt4, plot.colour, plot.thickness, plot.lineType, plot.shift);
                }
                if( plot.type == LINE_SEGMENTS) {
                    homo_.meter2Pix(plot.points[i], pt0);
                    homo_.meter2Pix(plot.points[i+1], pt1);
                    cv::line(view_, pt0, pt1, plot.colour, plot.thickness, plot.lineType, plot.shift);
                }
            }
        }
    }
}
