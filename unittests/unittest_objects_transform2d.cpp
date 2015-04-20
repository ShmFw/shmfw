
#include "unittest_objects.h"

namespace ShmFwTest {

TEST_F ( ObjectTest, Transform2DSerializeXML ) {
    std::string filename ( "Transform2D.xml" );
    ShmFw::Transform2D a ( randf(), randf(), randf() );
    ShmFw::Transform2D b;
    ShmFw::write ( filename, a );
    ShmFw::read ( filename, b );
    EXPECT_EQ ( a, b );
}
TEST_F ( ObjectTest, Transform2DSerializeTXT ) {
    std::string filename ( "Transform2D.txt" );
    ShmFw::Transform2D a ( randf(), randf(), randf() );
    ShmFw::Transform2D b;
    ShmFw::write ( filename, a );
    ShmFw::read ( filename, b );
    EXPECT_EQ ( a, b );
}

TEST_F ( ObjectTest, Transform2DSerializeOperator ) {
    ShmFw::Transform2D a ( randf(), randf(), randf() );
    ShmFw::Transform2D b;
    std::stringstream ss;
    ss << a;
    ss >> b;
    //std::cout << a << " = " << b << std::endl;
    EXPECT_EQ ( true, a.equal ( b, 0.1f ) ) << "Should be equal";
}
TEST_F ( ObjectTest, Transform2DOperationsPoint2D ) {
    ShmFw::Point2D p0 ( 1,0 );
    ShmFw::Transform2D tf ( 0, 0, M_PI );
    ShmFw::Point2D p1 = tf*p0;
    EXPECT_EQ ( true, p1.equal ( ShmFw::Point2D ( -1,0 ) ) ) << "Should be equal";
}

TEST_F ( ObjectTest, Transform2DOrientation ) {
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
TEST_F ( ObjectTest, Transform2DOperationsPose2DtoTf ) {
    ShmFw::Point2D p_r(1.05,1.8);
    //std::cout << p_r << std::endl; 
    ShmFw::Pose2D P_r(3.5,-4.5,-M_PI/4.);
    //std::cout << P_r << std::endl; 
    ShmFw::Transform2D tf_rw(P_r); 
    //std::cout << tf_rw << std::endl; 
    ShmFw::Point2D p_w = tf_rw * p_r;
    //std::cout << p_w << std::endl; 
    EXPECT_EQ ( true, p_w.equal ( ShmFw::Point2D ( 5.5, -4 ), 0.3) ) << "Should be equal";
    p_r = tf_rw / p_w;
    //std::cout << p_r << std::endl; 
    EXPECT_EQ ( true, p_r.equal ( ShmFw::Point2D ( 1.05,1.8 ), 0.1) ) << "Should be equal";
    ShmFw::Transform2D tf_wr = tf_rw.invert();
    p_r = tf_wr * p_w;
    //std::cout << p_r << std::endl; 
    EXPECT_EQ ( true, p_r.equal ( ShmFw::Point2D ( 1.05,1.8 ), 0.1) ) << "Should be equal";
    
}
}  // namespace
