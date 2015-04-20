
#include "unittest_objects.h"

namespace ShmFwTest{

TEST_F ( ObjectTest, serializeTwistXML ) {
    std::string filename ( "twist.xml" );
    ShmFw::Twist a(randf(), randf(), randf(), randf(), randf(), randf());
    ShmFw::Twist b;
    ShmFw::write(filename, a);
    ShmFw::read(filename, b); 
    EXPECT_EQ ( a, b );
}
TEST_F ( ObjectTest, serializeTwistTXT ) {
    std::string filename ( "twist.txt" );
    ShmFw::Twist a(randf(), randf(), randf(), randf(), randf(), randf());
    ShmFw::Twist b;
    ShmFw::write(filename, a);
    ShmFw::read(filename, b);
    EXPECT_EQ ( a, b );
}

TEST_F ( ObjectTest, serializeOperatorTwist ) {
    ShmFw::Twist a(randf(), randf(), randf(), randf(), randf(), randf());
    ShmFw::Twist b;
    std::stringstream ss;
    ss << a;
    ss >> b;
    //std::cout << a << " = " << b << std::endl;
    EXPECT_EQ ( true, a.equal ( b, 0.1f ) ) << "Should be equal";
}

}  // namespace
