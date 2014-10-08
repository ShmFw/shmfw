
#include "unittest_objects.h"
namespace ShmFwTest {


TEST_F ( ObjectTest, MakerShm ) {
    static const unsigned int N = 10;
    ShmFw::Vec<float, N> var1, var2;
    var1.rand ( -10, 10 );
    std::string str = var1.human_readable();
    var2.human_readable ( str );
    //std::cout << "var1: " << str << std::endl;
    //std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( true, var2.equal ( var1, 0.1f ) );
}

}  // namespace

