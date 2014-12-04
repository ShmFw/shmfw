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

#include <iostream>
#include <boost/lexical_cast.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef MX_CVFIGURE_H
#define MX_CVFIGURE_H



namespace ShmFw {


class Figure {
public:
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

    static const int LINE = 0;
    static const int DOT = 1;
    static const int DOT4 = 2;
    static const int DOT8 = 3;
    static const int CIRCLE = 4;
    static const int CROSS = 5;
    static const int LINE_SEGMENTS = 20;

    static cv::Scalar getColour ( int i );

    struct Homography {
        cv::Matx33d M;
        void init ( double a00, double a01, double a02, double a10, double a11, double a12, double angle );
        cv::Point &meter2Pix ( const cv::Point_<double> &src, cv::Point &des );
        cv::Point  meter2Pix ( const cv::Point_<double> &src );
        cv::Point  meter2Pix ( double x, double y );
    };
    struct Parameters {
        Parameters() {
            name = "CVFigure";
            widthPix = 600, heightPix = 600;
            auto_size = true;
            angle = 0;
            render_delay = 2;
            grid_dx = grid_dy = 1;
            grid_x_max = grid_y_max = grid_x_min = grid_y_min = 0;
            background = cv::Scalar ( 0xFF,0xFF,0xFF );
        }
        std::string name;
        unsigned int widthPix;
        unsigned int heightPix;
        bool auto_size;
        double angle;
        int render_delay;
        cv::Scalar background;
        double x_max, y_max, x_min, y_min;
        double grid_dx, grid_dy;
        double grid_x_max, grid_y_max, grid_x_min, grid_y_min;
        void setRange ( double _x_max, double _x_min, double _y_max, double _y_min, double _angle = 0 );
        void setGrid ( double dx, double dy, double _grid_x_max = 0, double _grid_x_min = 0, double _grid_y_max = 0, double _grid_y_min = 0 );
        void setWindowSize ( unsigned int widht, unsigned height );
    };

    struct PlotSyle {
        PlotSyle() {
            type = 0;
            colour =  cv::Scalar ( 0x00,0x00,0x00 );
            thickness = 1;
            lineType = 8;
            shift = 0;
        }
        void setValues(int _type, const cv::Scalar &_colour = cv::Scalar ( 0x00,0x00,0x00 ), int _thickness= 1, int _lineType = 8, int _shift = 0);
        int type;
        cv::Scalar colour;
        int thickness;
        int lineType;
        int shift;
    };
    struct Plot {
        Plot() {
            valid = true;
            name = "";
            type_param = 1;
            step_size = 1;
        }
        bool valid;
        std::string name;
        PlotSyle style;
        int type_param;
        int step_size;
        double x_max, y_max, x_min, y_min;
        std::vector<cv::Point_<double> > points;
        void findMaxima() {
            if ( points.size() > 0 ) {
                x_max = x_min = points[0].x, y_max = y_min = points[0].y;
            }
            for ( unsigned int i = 1; i < points.size(); i++ ) {
                cv::Point_<double> point = points[i];
                if ( x_max < point.x ) x_max = point.x;
                if ( y_max < point.y ) y_max = point.y;
                if ( x_min > point.x ) x_min = point.x;
                if ( y_min > point.y ) y_min = point.y;
            }
        }
    };

    Figure ();
    ~Figure();
    Parameters &getParam();
    void init();
    void render();

    void grid();

    void draw();

    Figure::Plot &getPlot ( const std::string &name );
    template <class T> Plot &addPlot ( const std::string &name, const std::vector<T> &x, const std::vector<T> &y ) {
        Plot &plot = plots_[name];
        if ( x.size() != y.size() ) {
            plot.valid = false;
            plot.points.resize ( 0 );
        }
        plot.points.resize ( x.size() );
        for ( unsigned int i = 0; i < plot.points.size(); i++ ) {
            plot.points[i] = cv::Point_<double> ( x[i], y[i] );
        }
        return plot;
    }
    template <class T> Plot &addPlot (const std::vector<T> &x, const std::vector<T> &y  ) {
      return addPlot(boost::lexical_cast<std::string>(plots_.size()), x, y);
    }
    template <class T> Plot &addSegments ( const std::string &name, const std::vector<T> &segments ) {
        Plot &plot = plots_[name];
        plot.step_size = 2;
        plot.points.resize ( segments.size() *plot.step_size );
        for ( unsigned int i = 0, j = 0; i < segments.size(); i++ ) {
            plot.points[j++] = cv::Point_<double> ( segments[i].x0, segments[i].y0 );
            plot.points[j++] = cv::Point_<double> ( segments[i].x1, segments[i].y1 );
        }
        plot.style.type = LINE_SEGMENTS;
        return plot;
    }
    template <class T> Plot &addSegments (const std::vector<T> &segments ) {
      return addSegments(boost::lexical_cast<std::string>(plots_.size()), segments);
    }
    template <class T> Plot &addSegments ( const std::string &name, const std::vector<T> &start_point, const std::vector<T> &end_point ) {
        Plot &plot = plots_[name];
        if ( start_point.size() != end_point.size() ) {
            plot.valid = false;
            plot.points.resize ( 0 );
        }
        plot.step_size = 2;
        plot.points.resize ( start_point.size() *plot.step_size );
        for ( unsigned int i = 0, j = 0; i < start_point.size(); i++ ) {
            plot.points[j++] = cv::Point_<double> ( start_point[i].x, start_point[i].y );
            plot.points[j++] = cv::Point_<double> ( end_point[i].x, end_point[i].y );
        }
        plot.style.type = LINE_SEGMENTS;
        return plot;
    }
    template <class T> Plot &addSegments (const std::vector<T> &start_point, const std::vector<T> &end_point ) {
      return addSegments(boost::lexical_cast<std::string>(plots_.size()), start_point, end_point);
    }
    template <class T> Plot &addPlot ( const std::string &name, const std::vector<T> &points ) {
        Plot &plot = plots_[name];
        plot.points.resize ( points.size() );
        for ( unsigned int i = 0; i < plot.points.size(); i++ ) {
            plot.points[i] = cv::Point_<double> ( points[i].x, points[i].y );
        }
        return plot;
    }
    template <class T> Plot &addPlot (const std::vector<T> &poses ) {
      return addPlot(boost::lexical_cast<std::string>(plots_.size()), poses);
    }

    template <class T> Plot &addPoses ( const std::string &name, const std::vector<T> &poses ) {
        Plot &plot = plots_[name];
        plot.points.resize ( poses.size() );
        for ( unsigned int i = 0; i < plot.points.size(); i++ ) {
            plot.points[i] = cv::Point_<double> ( poses[i].x(), poses[i].y() );
        }
        return plot;
    }
    template <class T> Plot &addPoses (const std::vector<T> &poses ) {
      return addPoses(boost::lexical_cast<std::string>(plots_.size()), poses);
    }


protected:
    unsigned long loop_count_;
    Parameters param_;
    cv::Mat view_;
    std::map<std::string, Plot> plots_;
    Homography homo_;
    void setDot ( int x, int y, const cv::Scalar &color ) {
        cv::Vec<char, 3> &pix = view_.at<cv::Vec<char, 3> > ( y, x );
        pix[0] = color[0], pix[1] = color[1], pix[2] = color[2];
    };
};
};
#endif // MX_CVFIGURE_H
