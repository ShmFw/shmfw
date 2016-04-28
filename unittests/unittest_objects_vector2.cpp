
#include "unittest_objects.h"
namespace ShmFwTest {


TEST_F ( ObjectTest, serializeToStringVector2 ) {
    ShmFw::Vector2<> var1, var2;
    var1.rand ( -10, 10 );
    std::string str = var1.human_readable();
    var2.human_readable ( str );
    //std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( true, var2.equal ( var1,0.1 ) );
}

TEST_F ( ObjectTest, serializeVector2 ) {
    std::string filename ( "/tmp/vec2.xml" );
    ShmFw::Vector2<> var1, var2;
    var1.rand ( -10, 10 );
    ShmFw::write ( filename, var1, ShmFw::FORMAT_XML );
    ShmFw::read ( filename, var2, ShmFw::FORMAT_XML );
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( var1, var2 );
}


TEST_F ( ObjectTest, operationsVectorVector2 ) {
    {
        ShmFw::Vector2<int> varResult ( 10, 0 ), varComputed;
        varComputed = ShmFw::Vector2<int> ( 20, 10 ) - ShmFw::Vector2<int> ( 10, 10 );
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector2<double> varResult ( 30, 80.2 ), varComputed;
        varComputed = ShmFw::Vector2<double> ( 20, 30 ) + ShmFw::Vector2<double> ( 10, 50.2 );
        EXPECT_EQ ( true, varComputed == varComputed );
    }
}

TEST_F ( ObjectTest, operationsDotVector2 ) {
    {
        int varComputed = ShmFw::Vector2<int> ( 20, 30 )  * ShmFw::Vector2<int> ( 10, 50 );
        EXPECT_EQ ( 1700, varComputed );
    }
    {
        float varComputed = ShmFw::Vector2<float> ( 20, 30 )  * ShmFw::Vector2<float> ( 10, 50.1 );
        EXPECT_EQ ( 1703, varComputed );
    }
}


TEST_F ( ObjectTest, equalVector2Int ) {
    ShmFw::Vector2<int> a ( 0, 0 );
    ShmFw::Vector2<int> b ( 2, 0 );
    bool similar = a.equal ( b, 3 );
    EXPECT_TRUE ( similar ) << "Should be equal";
    similar = a.equal ( b, 1 );
    EXPECT_FALSE ( similar ) << "Should no be equal";
}


TEST_F ( ObjectTest, equalVector2Double ) {
    ShmFw::Vector2<double> a ( 0, 0 );
    ShmFw::Vector2<double> b ( 1, 1 );
    bool similar = a.equal ( b, sqrt(2)+0.1 );
    EXPECT_TRUE ( similar ) << "Should be equal";
    similar = a.equal ( b, sqrt(2) );
    EXPECT_FALSE ( similar ) << "Should no be equal";
}

TEST_F ( ObjectTest, operationsScalarVector2 ) {
    {
        ShmFw::Vector2<int> varResult ( 10, 0 ), varComputed;
        varComputed = ShmFw::Vector2<int> ( 20, 10 ) - ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<int> ( 20, 10 ) - ( float ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<int> ( 20, 10 ) - ( double ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector2<float> varResult ( 10.1, 0.2 ), varComputed;
        varComputed = ShmFw::Vector2<float> ( 20.1, 10.2 ) - ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<float> ( 20.1, 10.2 ) - ( float ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<float> ( 20.1, 10.2 ) - ( double ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector2<double> varResult ( 10.1, 0.2 ), varComputed;
        varComputed = ShmFw::Vector2<double> ( 20.1, 10.2 ) - ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<double> ( 20.1, 10.2 ) - ( float ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<double> ( 20.1, 10.2 ) - ( double ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector2<int> varResult ( 20, 20 ), varComputed;
        varComputed = ShmFw::Vector2<int> ( 10, 10 ) + ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<int> ( 10, 10 ) + ( float ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<int> ( 10, 10 ) + ( double ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector2<float> varResult ( 20.1, 20.2 ), varComputed;
        varComputed = ShmFw::Vector2<float> ( 10.1, 10.2 ) + ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<float> ( 10.1, 10.2 ) + ( float ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<float> ( 10.1, 10.2 ) + ( double ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector2<double> varResult ( 20.1, 20.2 ), varComputed;
        varComputed = ShmFw::Vector2<double> ( 10.1, 10.2 ) + ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<double> ( 10.1, 10.2 ) + ( float ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<double> ( 10.1, 10.2 ) + ( double ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector2<int> varResult ( 30, 45 ), varComputed;
        varComputed = ShmFw::Vector2<int> ( 6, 9 ) * ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<int> ( 6, 9 ) * ( float ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<int> ( 6, 9 ) * ( double ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector2<float> varResult ( 30, 45 ), varComputed;
        varComputed = ShmFw::Vector2<float> ( 6, 9 ) * ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<float> ( 20, 30 ) * ( float ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<float> ( 20, 30 ) * ( double ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector2<double> varResult ( 30, 45 ), varComputed;
        varComputed = ShmFw::Vector2<double> ( 6, 9 ) * ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<double> ( 20, 30 ) * ( float ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<double> ( 20, 30 ) * ( double ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector2<int> varResult ( 6, 9 ), varComputed;
        varComputed = ShmFw::Vector2<int> ( 30, 45 ) / ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<int> ( 30, 45 ) / ( float ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<int> ( 30, 45 ) / ( double ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector2<float> varResult ( 6, 9 ), varComputed;
        varComputed = ShmFw::Vector2<float> ( 30, 45 ) / ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<float> ( 9, 13.5 ) / ( float ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<float> ( 9, 13.5 ) / ( double ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector2<double> varResult ( 6, 9 ), varComputed;
        varComputed = ShmFw::Vector2<double> ( 30, 45 ) / ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<double> ( 9, 13.5 ) / ( float ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector2<double> ( 9, 13.5 ) / ( double ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
}
TEST_F ( ObjectTest, copyToFromVector2 ) {
    ShmFw::Point2D a ( rand(), rand() ), c;
    ShmFw::Vector2<double> b, d;
    b.copyFrom ( a );
    b.copyTo ( d );
    c.copyFrom ( d );
    c.copyTo ( d );
    std::cout << "a: " << a << ", d: " << d << std::endl;
    EXPECT_EQ ( a, c );
    EXPECT_EQ ( b, d );
}
}  // namespace

