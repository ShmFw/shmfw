
#include "unittest_objects.h"
namespace ShmFwTest{

TEST_F ( ObjectTest, serializeToStringVector3 ) {
    ShmFw::Vector3<> var1, var2;
    var1.rand ( -10, 10 );
    std::string str = var1.human_readable();
    var2.human_readable(str);
    //std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( true, var2.equal(var1,0.1) );
}

TEST_F ( ObjectTest, serializeVector3 ) {
    std::string filename ( "vec3.xml" );
    ShmFw::Vector3<> var1, var2;
    var1.rand ( -10, 10 );
    ShmFw::write ( filename, var1, ShmFw::FORMAT_XML );
    ShmFw::read ( filename, var2, ShmFw::FORMAT_XML );
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( var1, var2 );
}

TEST_F ( ObjectTest, operationsVectorVector3 ) {
    {
        ShmFw::Vector3<int> varResult ( 10, 0, -20 ), varComputed;
        varComputed = ShmFw::Vector3<int> ( 20, 10, 30 ) - ShmFw::Vector3<int> ( 10, 10, 50 );
        EXPECT_EQ ( true, varComputed == varComputed );
	}
    {
        ShmFw::Vector3<double> varResult ( 30, 0, 80.2 ), varComputed;
        varComputed = ShmFw::Vector3<double> ( 20, 10, 30 ) + ShmFw::Vector3<double> ( 10, -10, 50.2 );
        EXPECT_EQ ( true, varComputed == varComputed );
	}
}

TEST_F ( ObjectTest, operationsDotVector3 ) {
    {
        int varComputed = ShmFw::Vector3<int> ( 20, 10, 30 )  * ShmFw::Vector3<int> ( 10, 10, 50 );
        EXPECT_EQ ( 1800, varComputed );
	}
    {
        float varComputed = ShmFw::Vector3<float> ( 20, 10, 30 )  * ShmFw::Vector3<float> ( 10, 10, 50.1 );
        EXPECT_EQ ( 1803, varComputed );
	}
}

TEST_F ( ObjectTest, operationsCrossVector3 ) {
    {
        ShmFw::Vector3<double> varResult ( 200, -700, 100 ), varComputed;
        varComputed = ShmFw::Vector3<double> ( 20, 10, 30 ) ^ (ShmFw::Vector3<double> ( 10, 10, 50 ));
        EXPECT_EQ ( true, varComputed == varResult);
	}
}
TEST_F ( ObjectTest, operationsScalarVector3 ) {
    {
        ShmFw::Vector3<int> varResult ( 10, 0, 20 ), varComputed;
        varComputed = ShmFw::Vector3<int> ( 20, 10, 30 ) - ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<int> ( 20, 10, 30 ) - ( float ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<int> ( 20, 10, 30 ) - ( double ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
    }  
    {
        ShmFw::Vector3<float> varResult ( 10.1, 0.2, 20.3 ), varComputed;
        varComputed = ShmFw::Vector3<float> ( 20.1, 10.2, 30.3 ) - ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<float> ( 20.1, 10.2, 30.3 ) - ( float ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<float> ( 20.1, 10.2, 30.3 ) - ( double ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
    }   
    {
        ShmFw::Vector3<double> varResult ( 10.1, 0.2, 20.3 ), varComputed;
        varComputed = ShmFw::Vector3<double> ( 20.1, 10.2, 30.3 ) - ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<double> ( 20.1, 10.2, 30.3 ) - ( float ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<double> ( 20.1, 10.2, 30.3 ) - ( double ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector3<int> varResult ( 20, 20, 20 ), varComputed;
        varComputed = ShmFw::Vector3<int> ( 10, 10, 10 ) + ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<int> ( 10, 10, 10 ) + ( float ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<int> ( 10, 10, 10 ) + ( double ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector3<float> varResult ( 20.1, 20.2, 20.3 ), varComputed;
        varComputed = ShmFw::Vector3<float> ( 10.1, 10.2, 10.3 ) + ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<float> ( 10.1, 10.2, 10.3 ) + ( float ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<float> ( 10.1, 10.2, 10.3 ) + ( double ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector3<double> varResult ( 20.1, 20.2, 20.3 ), varComputed;
        varComputed = ShmFw::Vector3<double> ( 10.1, 10.2, 10.3 ) + ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<double> ( 10.1, 10.2, 10.3 ) + ( float ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<double> ( 10.1, 10.2, 10.3 ) + ( double ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector3<int> varResult ( 30, 45, 60 ), varComputed;
        varComputed = ShmFw::Vector3<int> ( 6, 9, 12 ) * ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<int> ( 6, 9, 12 ) * ( float ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<int> ( 6, 9, 12 ) * ( double ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector3<float> varResult ( 30, 45, 60 ), varComputed;
        varComputed = ShmFw::Vector3<float> ( 6, 9, 12 ) * ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<float> ( 20, 30, 40 ) * ( float ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<float> ( 20, 30, 40 ) * ( double ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector3<double> varResult ( 30, 45, 60 ), varComputed;
        varComputed = ShmFw::Vector3<double> ( 6, 9, 12 ) * ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<double> ( 20, 30, 40 ) * ( float ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<double> ( 20, 30, 40 ) * ( double ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector3<int> varResult ( 6, 9, 12 ), varComputed;
        varComputed = ShmFw::Vector3<int> ( 30, 45, 60 ) / ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<int> ( 30, 45, 60 ) / ( float ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<int> ( 30, 45, 60 ) / ( double ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector3<float> varResult ( 6, 9, 12 ), varComputed;
        varComputed = ShmFw::Vector3<float> ( 30, 45, 60 ) / ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<float> ( 9, 13.5, 18 ) / ( float ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<float> ( 9, 13.5, 18 ) / ( double ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector3<double> varResult ( 6, 9, 12 ), varComputed;
        varComputed = ShmFw::Vector3<double> ( 30, 45, 60 ) / ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<double> ( 9, 13.5, 18 ) / ( float ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector3<double> ( 9, 13.5, 18 ) / ( double ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
}
TEST_F ( ObjectTest, equalVector3Int ) {
    ShmFw::Vector3<int> a ( 0, 0, 0 );
    ShmFw::Vector3<int> b ( 2, 0, 0 );
    bool similar = a.equal ( b, 3 );
    EXPECT_TRUE ( similar ) << "Should be equal";
    similar = a.equal ( b, 1 );
    EXPECT_FALSE ( similar ) << "Should no be equal";
}


TEST_F ( ObjectTest, equalVector3Double ) {
    ShmFw::Vector3<double> a ( 0, 0, 0 );
    ShmFw::Vector3<double> b ( 1, 1, 0 );
    bool similar = a.equal ( b, sqrt(2)+0.1 );
    EXPECT_TRUE ( similar ) << "Should be equal";
    similar = a.equal ( b, sqrt(2) );
    EXPECT_FALSE ( similar ) << "Should no be equal";
}
TEST_F ( ObjectTest, copyToFromVector3 ) {
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

