#include <iostream>
#include "shmfw/figure.h"
#include "shmfw/variable.h"
#include "shmfw/vector.h"
#include "shmfw/objects/vector2.h"
#include "shmfw/deque.h"
#include "shmfw/log.h"
#include <math.h>

int main(int argc, char **argv) {
  /*
    std::cout << "Hello, Figure!" << std::endl;
    
    std::vector<ShmFw::Vector2<double> > points;
    for(double x = -10.; x < 10.; x += 0.2){
      points.push_back(ShmFw::Vector2<double>(x, cos(x)));
    }
    
    ShmFw::Figure figure;
    figure.getParam().setWindowSize(1024,1024);
    figure.init();
    ShmFw::Figure::Plot &plot_scan = figure.addPlot("scan", points);
    plot_scan.type = ShmFw::Figure::LINE;
    plot_scan.colour = ShmFw::Figure::getColour(ShmFw::Figure::RED);
    figure.render();
    cv::waitKey(-1);
    */
  
    ShmFw::Figure figure;
    
    figure.getParam().setWindowSize(640, 480);
    figure.getParam().setRange(5,-1, 5, -5);
    figure.init();  
    figure.render();
    cv::waitKey(-1);
    
    return 0;
}
