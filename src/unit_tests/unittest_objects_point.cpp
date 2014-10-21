
#include "unittest_objects.h"

namespace ShmFwTest{

TEST_F ( ObjectTest, serializePoint ) {
    std::string filename ( "planePoint.xml" );
    ShmFw::Point p1, p2;
    p1.x = rand(), p1.y = rand(), p1.z = rand();
    ShmFw::write(filename, p1);
    ShmFw::read (filename, p2);
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( p1, p2 );
}
TEST_F ( ObjectTest, copyToFromPoint ) {
    ShmFw::Point a(rand(), rand(), rand()), c;
    ShmFw::Vector3<double> b, d;
    b.copyFrom(a);
    b.copyTo(d);
    c.copyFrom(d);
    c.copyTo(d);
    std::cout << "a: " << a << ", d: " << d << std::endl;
    EXPECT_EQ ( a, c );
    EXPECT_EQ ( b, d );
}

}  // namespace
