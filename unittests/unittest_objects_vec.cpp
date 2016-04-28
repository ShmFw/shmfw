
#include "unittest_objects.h"
namespace ShmFwTest {


TEST_F ( ObjectTest, serializeToStringVec ) {
    static const unsigned int N = 10;
    ShmFw::Vec<float, N> var1, var2;
    var1.rand ( -10, 10 );
    std::string str = var1.human_readable();
    var2.human_readable ( str );
    std::cout << "var1: " << str << std::endl;
    std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( true, var2.equal ( var1, 0.1f ) );
}


TEST_F ( ObjectTest, serializeVec) {
    static const unsigned int N = 10;
    std::string filename ( "/tmp/vec.xml" );
    ShmFw::Vec<float, N> var1, var2;
    var1.rand ( -10, 10 );
    ShmFw::write ( filename, var1, ShmFw::FORMAT_XML );
    ShmFw::read ( filename, var2, ShmFw::FORMAT_XML );
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( var1, var2 );
}

TEST_F ( ObjectTest, operationsVectorVec ) {
    static const unsigned int N = 10;
    ShmFw::Vec<float, N> varResult, varComputed;
    ShmFw::Vec<float, N> var1, var2;
    var1.rand ( -10, 10 );
    for ( unsigned int i = 0; i < N; i++ ) varResult[i] = var1[i] + var2[i];
    varComputed = var1 + var2;
    EXPECT_EQ ( varResult, varComputed );
}

TEST_F ( ObjectTest, operationsDotVec ) {
    static const unsigned int N = 10;
    float varResult=0, varComputed;
    ShmFw::Vec<float, N> var1, var2;
    var1.rand ( -10, 10 );
	
    for ( unsigned int i = 0; i < N; i++ ) varResult += var1[i] * var2[i];
    varComputed = var1 * var2;
    EXPECT_EQ ( varResult, varComputed );
}

TEST_F ( ObjectTest, operationsCrossVec ) {
    /*
      {
          ShmFw::Vec<double> varResult ( 200, -700, 100 ), varComputed;
          varComputed = ShmFw::Vec<double> ( 20, 10, 30 ) ^ (ShmFw::Vec<double> ( 10, 10, 50 ));
          EXPECT_EQ ( true, varComputed == varResult);
    }
    */
}

TEST_F ( ObjectTest, operationsScalarVec ) {
    static const unsigned int N = 10;
    ShmFw::Vec<float, N> varResult, varComputed;
    ShmFw::Vec<float, N> var;
	float scalar = 123123.234;
    for ( unsigned int i = 0; i < N; i++ ) varResult[i] = var[i] - scalar;
    varComputed = var - scalar;
    EXPECT_EQ ( varResult, varComputed );
    for ( unsigned int i = 0; i < N; i++ ) varResult[i] = var[i] + scalar;
    varComputed = var + scalar;
    EXPECT_EQ ( varResult, varComputed );
    for ( unsigned int i = 0; i < N; i++ ) varResult[i] = var[i] * scalar;
    varComputed = var * scalar;
    EXPECT_EQ ( varResult, varComputed );
    for ( unsigned int i = 0; i < N; i++ ) varResult[i] = var[i] / scalar;
    varComputed = var / scalar;
    EXPECT_EQ ( varResult, varComputed );
}
}  // namespace

