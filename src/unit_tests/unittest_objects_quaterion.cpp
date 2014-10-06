
#include "unittest_objects.h"

namespace ShmFwTest{

TEST_F ( ObjectTest, serializeQuaterion ) {
    std::string filename ( "quaterion.xml" );
    ShmFw::Quaternion a(randf(), randf(), rand(), rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}

}  // namespace
