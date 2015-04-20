
#include "unittest_objects.h"

namespace ShmFwTest {

TEST_F ( ObjectTest, serializeTransform2dXML ) {
    std::string filename ( "transform2d.xml" );
    ShmFw::Transform2D a ( randf(), randf(), randf() );
    ShmFw::Transform2D b;
    ShmFw::write ( filename, a );
    ShmFw::read ( filename, b );
    EXPECT_EQ ( a, b );
}
TEST_F ( ObjectTest, serializeTransform2dTXT ) {
    std::string filename ( "transform2d.txt" );
    ShmFw::Transform2D a ( randf(), randf(), randf() );
    ShmFw::Transform2D b;
    ShmFw::write ( filename, a );
    ShmFw::read ( filename, b );
    EXPECT_EQ ( a, b );
}

TEST_F ( ObjectTest, serializeOperatorTransform2d ) {
    ShmFw::Transform2D a ( randf(), randf(), randf() );
    ShmFw::Transform2D b;
    std::stringstream ss;
    ss << a;
    ss >> b;
    //std::cout << a << " = " << b << std::endl;
    EXPECT_EQ ( true, a.equal ( b, 0.1f ) ) << "Should be equal";
}
TEST_F ( ObjectTest, transform2Operations ) {
    ShmFw::Point2D p0 ( 1,0 );
    ShmFw::Transform2D tf ( 0, 0, M_PI );
    ShmFw::Point2D p1 = tf*p0;
    EXPECT_EQ ( true, p1.equal ( ShmFw::Point2D ( -1,0 ) ) ) << "Should be equal";
}

TEST_F ( ObjectTest, transform2dOrientation ) {
    double a0, a1;
    ShmFw::Transform2D tf;
    a0 = randf ( 0, M_PI/2 );
    tf.setTf ( 0, 0, a0 );
    a1 = tf.orientation();
    EXPECT_NEAR ( a0, a1, 0.0001 );
    a0 = randf ( M_PI/2, M_PI );
    tf.setTf ( 0, 0, a0 );
    a1 = tf.orientation();
    EXPECT_NEAR ( a0, a1, 0.0001 );
    a0 = randf ( -M_PI, -M_PI/2 );
    tf.setTf ( 0, 0, a0 );
    a1 = tf.orientation();
    EXPECT_NEAR ( a0, a1, 0.0001 );
    a0 = randf ( -M_PI/2, 0 );
    tf.setTf ( 0, 0, a0 );
    a1 = tf.orientation();
    EXPECT_NEAR ( a0, a1, 0.0001 );
}
}  // namespace
