
#include "unittest_objects.h"
#include "shmfw/objects/line2d.h"
namespace ShmFwTest {


TEST_F ( ObjectTest, operationsLineDistanceTo ) {
  
    ShmFw::Line2D<double> l(0,0,1,1);
    double d = l.distanceTo(cv::Point2d(0,1));    
    EXPECT_NEAR(d, 0.7, 0.1);
    d = l.distanceTo(cv::Point2d(1,0));
    EXPECT_NEAR(d, -0.7, 0.1);
}


TEST_F ( ObjectTest, operationsLineIntersection ) {
  
    ShmFw::Line2D<double> l1(0,0,1,1);
    ShmFw::Line2D<double> l2(1,0,0,1);
    cv::Point_<double> p0 = l1.intersection(l2);    
    cv::Point_<double> p(0.5,0.5);    
    EXPECT_EQ(p0, p);
}


TEST_F ( ObjectTest, serializeLine ) {
    std::string filename ( "line.xml" );
    ShmFw::Line2D<double> a(rand_01(),rand_01(),rand_01(),rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}

}  // namespace

