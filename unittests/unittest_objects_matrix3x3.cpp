
#include "unittest_objects.h"

namespace ShmFwTest{


TEST_F ( ObjectTest, serializeMatrix3x3 ) {
    std::string filename ( "/tmp/mat3x3.xml" );
    ShmFw::Matrix3x3<> var1, var2;
    var1.rand ( -10, 10 );
    ShmFw::write ( filename, var1, ShmFw::FORMAT_XML );
    ShmFw::read ( filename, var2, ShmFw::FORMAT_XML );
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( var1, var2 );
}

TEST_F ( ObjectTest, operationsMatrix3x3 ) {
    std::string filename ( "/tmp/mat3x3.xml" );
    ShmFw::Matrix3x3<> varEye, var1, var2, var3;
    varEye.eye();
    var1.rand ( -10, 10 );
    var2 = varEye * var1;
//     std::cout << "var1: \n" << var1.human_readable() << std::endl;
//     std::cout << "var2: \n" << var2.human_readable() << std::endl;
//     std::cout << "varEye: \n" << varEye.human_readable() << std::endl;
    EXPECT_EQ ( var1, var2 );
    var3 = varEye;
    var3 *= var1;
    EXPECT_EQ ( var1, var3 );
}

TEST_F ( ObjectTest, operationsVectorMatrix3x3 ) {
    std::string filename ( "/tmp/mat3x3.xml" );
    ShmFw::Matrix3x3<> varEye, mat1, mat2, mat3;
    ShmFw::Vector3<> vec1, vec2;
    varEye.eye();
    vec1.rand ( -10, 10 );
    vec2 = varEye * vec1;
    EXPECT_EQ ( vec1, vec2 );
}

/*
TEST_F ( ObjectTest, invertMatrix3x3 ) {
    std::string filename ( "mat3x3.xml" );
    ShmFw::Matrix3x3<> varEye, mat1, mat2, mat3;
    ShmFw::Vector3<> vec1, vec2, vec3;
    mat1.rand ( -10, 10 );
    vec1.rand ( -10, 10 );
    vec2 = mat1 * vec1;
    mat2 = mat1.invert();
    std::cout << "mat1: \n" << mat1.human_readable() << std::endl;
    std::cout << "det mat1: \n" << mat1.det() << std::endl;
    std::cout << "vec1: \n" << vec1.human_readable() << std::endl;
    std::cout << "vec2: \n" << vec2.human_readable() << std::endl;
    std::cout << "mat2: \n" << mat2.human_readable() << std::endl;
    vec3 = mat2 * vec2;
    EXPECT_EQ ( vec3, vec1 );
}
*/

}  // namespace
