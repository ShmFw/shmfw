
#include "unittest_objects.h"
#include "shmfw/objects/linesegment2d.h"
namespace ShmFwTest {


TEST_F ( ObjectTest, operationsLineSegment2Line ) {
  
    cv::Point_<double> p1(0,0), p2(1,1);
    ShmFw::LineSegment2D<double> s(p1,p2);
    double d1 = s.line().distanceTo(cv::Point2d(0,1)); 
    EXPECT_NEAR(d1, 0.7, 0.1);
    s.set(p2,p1);
    double d2 = s.line().distanceTo(cv::Point2d(0,1)); 
    EXPECT_NEAR(d2, -0.7, 0.1);
}


TEST_F ( ObjectTest, operationsLineLength ) {
  
    cv::Point_<double> p1(0,0), p2(1,1);
    ShmFw::LineSegment2D<double> s(p1,p2);
    double d = s.length(); 
    EXPECT_NEAR(d, 1.4, 0.1);
}
TEST_F ( ObjectTest, operationsLineSegment ) {
  
    cv::Point_<double> p1(0,0), p2(1,1);
    ShmFw::LineSegment2D<double> s(p1,p2);
    double d1 = s.distanceTo(cv::Point2d(0,1)); 
    EXPECT_NEAR(d1, 0.7, 0.1);
    double d2 = s.distanceTo(cv::Point2d(1,0)); 
    EXPECT_NEAR(d2, 0.7, 0.1);
    double d3 = s.distanceTo(cv::Point2d(2,2)); 
    EXPECT_NEAR(d3, 1.4, 0.1);
    double d4 = s.distanceTo(cv::Point2d(-1,-1)); 
    EXPECT_NEAR(d4, 1.4, 0.1);
}


TEST_F ( ObjectTest, serializeLineSegment ) {
    std::string filename ( "linesegment.xml" );
    cv::Point_<double> p1(0,0), p2(1,1);
    ShmFw::LineSegment2D<double> a(p1,p2), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}

}  // namespace

