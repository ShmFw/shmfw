
#include "unittest_objects.h"

namespace ShmFwTest{

  /*
TEST_F ( ObjectTest, serializeToStringPlaneEquation ) {
    ShmFw::PlaneEquation<> var1, var2;
    var1.rand ( -10, 10 );
    std::string str = var1.human_readable();
    var2.human_readable(str);
    //std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( true, var2.equal(var1,0.1) );
}
*/

TEST_F ( ObjectTest, serializePlaneEquation ) {
    std::string filename ( "/tmp/planeEquation.xml" );
    ShmFw::PlaneEquation<> var1, var2;
    var1.rand ( -10, 10 );
    ShmFw::write ( filename, var1, ShmFw::FORMAT_XML );
    ShmFw::read ( filename, var2, ShmFw::FORMAT_XML );
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( var1, var2 );
}

TEST_F ( ObjectTest, serializePlane ) {
    std::string filename ( "/tmp/plane.xml" );
    ShmFw::Plane<> var1, var2;
    var1.rand ( -10, 10 );
    ShmFw::write ( filename, var1, ShmFw::FORMAT_XML );
    ShmFw::read ( filename, var2, ShmFw::FORMAT_XML );
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( var1, var2 );
}

TEST_F ( ObjectTest, operationsPlaneDistanceToPoint ) {
    ShmFw::Plane<> plane;
    plane.eq.create ( ShmFw::Vector3<> ( 0,0,1 ),ShmFw::Vector3<> ( 0,4,1 ), ShmFw::Vector3<> ( 4,0,1 ) );
    float d1 = plane.eq.distanceToPlane ( ShmFw::Vector3<> ( 0,0,2 ) );
    EXPECT_EQ ( d1, -1 );
    plane.eq.create ( ShmFw::Vector3<> ( 234., 24.134, 96. ),ShmFw::Vector3<> ( 264., 27.14, 6.65 ), ShmFw::Vector3<> ( 64., 337.1, 226.56 ) );
    float d2 = plane.eq.distanceToPlane ( ShmFw::Vector3<> ( 23., 498.345, 394.4 ) );
    EXPECT_TRUE (( fabs ( 72.2153-d2 ) < 0.001 ) );
}

TEST_F ( ObjectTest, operationsPlaneLineIntersection ) {
    ShmFw::Plane<> plane;
    bool intersect;
    plane.eq.create ( ShmFw::Vector3<> ( 0,0,1 ),ShmFw::Vector3<> ( 0,4,1 ), ShmFw::Vector3<> ( 4,0,1 ) );
    ShmFw::Vector3<> intersection;
    intersect = plane.intersectionLine(ShmFw::Vector3<> ( 3,3,1 ), ShmFw::Vector3<> (2,55,1 ), intersection);
    EXPECT_FALSE ( intersect );
    intersect = plane.intersectionLine(ShmFw::Vector3<> ( 3.3,5.6,0 ), ShmFw::Vector3<> (3.3,5.6,5 ), intersection);
    EXPECT_TRUE ( intersect );
    EXPECT_EQ ( ShmFw::Vector3<> ( 3.3,5.6,1.0 ), intersection );
}

}  // namespace
