
#include "unittest_objects.h"
#include <shmfw/objects/matrix3x3.h>

namespace ShmFwTest{

TEST_F ( ObjectTest, serializeQuaterion ) {
    std::string filename ( "quaterion.xml" );
    ShmFw::Quaternion a(randf(), randf(), rand(), rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}

TEST_F ( ObjectTest, QuaterionRotations ) {
    double roll = randAngle();
    double pitch = randAngle();
    double yaw = randAngle();
    ShmFw::Quaternion a;
    a.setEuler(yaw, pitch, roll);
    ShmFw::Matrix3x3<double> R;
    R.setRotation(a.x, a.y, a.z, a.w);
    ShmFw::Quaternion b;
    R.getRotation(b.x, b.y, b.z, b.w);
    //std::cout << a << std::endl << b << std::endl;
    EXPECT_EQ ( true, a.equal ( b, 0.1f ) ) << "Should be equal";
}

}  // namespace
