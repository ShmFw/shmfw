

#include <iostream>
#include <string.h>
#include <vector>
#include <stdio.h>
#include <shmfw/vector.h>
#include <shmfw/deque.h>
#include <shmfw/objects/point2d.h>
#include <shmfw/objects/pose2d.h>
#include <boost/serialization/vector.hpp>
#include <shmfw/serialization/vector.h>
#include <shmfw/serialization/deque.h>
#include <shmfw/serialization/io_file.h>

int main() {
  
    int N = 10;
    boost::shared_ptr<ShmFw::Handler> shmHdl = ShmFw::Handler::create ( DEFAULT_SEGMENT_NAME, DEFAULT_SEGMENT_SIZE );
    
    
    
    std::string filePoints("/tmp/points.xml");
    std::cout << "-------------- Vector read/write --------------" << std::endl;
    std::cout << "to -> " << filePoints << std::endl;
    
    ShmFw::Vector<ShmFw::Point2D> shmPointsSrc ( "pointsSrc", shmHdl );
    ShmFw::Vector<ShmFw::Point2D> shmPointsDes ( "pointsDes", shmHdl );

    shmPointsSrc().clear();
    for ( int i = 0; i < N; i++ ) {
        shmPointsSrc.push_back(ShmFw::Point2D(rand(), rand()));
    }

    ShmFw::write(filePoints, shmPointsSrc, ShmFw::FORMAT_XML);
    ShmFw::read(filePoints, shmPointsDes, ShmFw::FORMAT_XML);
    
    if ( shmPointsSrc.size() == shmPointsDes.size() ) {
        for ( size_t i = 0; i < shmPointsDes.size(); i++ ) {
            std::cout << i << ": " << shmPointsDes[i] << " = " << shmPointsDes[i] << std::endl;
        }
    } else {
        std::cout << "Something went wrong!" << std::endl;
    }
    
    
    std::string filePoses("/tmp/poses.xml");
    std::cout << "-------------- Vector read/write --------------" << std::endl;
    std::cout << "to -> " << filePoses << std::endl;
    
    ShmFw::Deque<ShmFw::Pose2D> shmPosesSrc ( "posesSrc", shmHdl );
    ShmFw::Deque<ShmFw::Pose2D> shmPosesDes ( "posesDes", shmHdl );
    
    shmPosesSrc().clear();
    for ( int i = 0; i < N; i++ ) {
        shmPosesSrc.push_back(ShmFw::Pose2D(rand(), rand(), rand()));
    }
    
    
    ShmFw::write(filePoses, shmPosesSrc, ShmFw::FORMAT_XML);
    ShmFw::read(filePoses, shmPosesDes, ShmFw::FORMAT_XML);

    
    if ( shmPointsSrc.size() == shmPointsDes.size() ) {
        for ( size_t i = 0; i < shmPointsDes.size(); i++ ) {
            std::cout << i << ": " << shmPointsDes[i] << " = " << shmPointsDes[i] << std::endl;
        }
    } else {
        std::cout << "Something went wrong!" << std::endl;
    }
    
    return 0;
}
