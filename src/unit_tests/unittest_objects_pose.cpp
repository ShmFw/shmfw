
#include "unittest_objects.h"

namespace ShmFwTest{

TEST_F ( ObjectTest, convertPose2DtoPose3D ) {
    ShmFw::Pose2D p1(rand(),rand(),RandAngle());  p2;    
    p1.position = ShmFw::Point<double>(::rand(),::rand(),::rand());
    p1.orientation = ShmFw::Quaternion<double>(0,0,0,1);
    p1.write ( filename );
    p2.read ( filename );
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( p1, p2 );
}

TEST_F ( ObjectTest, serializePose ) {
    std::string filename ( "unittestPose.xml" );
    ShmFw::Pose<double> p1, p2;    
    p1.position = ShmFw::Point<double>(::rand(),::rand(),::rand());
    p1.orientation = ShmFw::Quaternion<double>(0,0,0,1);
    p1.write ( filename );
    p2.read ( filename );
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( p1, p2 );
}
TEST_F ( ObjectTest, serializePoseStamped ) {
    std::string filename ( "unittestPoseStamped.xml" );
    ShmFw::PoseStamped<double> p1, p2;    
    p1.position = ShmFw::Point<double>(::rand(),::rand(),::rand());
    p1.orientation = ShmFw::Quaternion<double>(0,0,0,1);
    p1.tstamp = boost::posix_time::microsec_clock::universal_time();
    p1.write ( filename );
    p2.read ( filename );
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    //EXPECT_EQ ( p1, p1 );
}
TEST_F ( ObjectTest, serializePose2D ) {
    std::string filename ( "unittestPose2D.xml" );
    ShmFw::Pose2D<double> p1, p2;    
    p1.position = ShmFw::Point2D<double>(::rand(),::rand());
    p1.orientation = ::rand();
    p1.write ( filename );
    p2.read ( filename );
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( p1, p2 );
}
TEST_F ( ObjectTest, serializePose2DStamped ) {
    std::string filename ( "unittestPose2DStamped.xml" );
    ShmFw::Pose2DStamped<double> p1, p2;    
    p1.position = ShmFw::Point2D<double>(::rand(),::rand());
    p1.orientation = ::rand();
    p1.tstamp = boost::posix_time::microsec_clock::universal_time();
    p1.write ( filename );
    p2.read ( filename );
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    //EXPECT_EQ ( p1, p1 );
}


}  // namespace
