
#include "unittest_objects.h"
namespace ShmFwTest{

  
TEST_F ( ObjectTest, serializeToStringVector4 ) {
    ShmFw::Vector4<> var1, var2;
    var1.rand ( -10, 10 );
    std::string str = var1.human_readable();
    var2.human_readable(str);
    //std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( true, var2.equal(var1,0.1) );
}

TEST_F ( ObjectTest, serializeVector4 ) {
    std::string filename ( "/tmp/vec3.xml" );
    ShmFw::Vector4<> var1, var2;
    var1.rand ( -10, 10 );
    ShmFw::write ( filename, var1, ShmFw::FORMAT_XML );
    ShmFw::read ( filename, var2, ShmFw::FORMAT_XML );
    // std::cout << "var1: " << var1 << ", var2: " << var2 << std::endl;
    EXPECT_EQ ( var1, var2 );
}

TEST_F ( ObjectTest, operationsVectorVector4 ) {
    {
        ShmFw::Vector4<int> varResult ( 10, 0, -20, -20 ), varComputed;
        varComputed = ShmFw::Vector4<int> ( 20, 10, 30, 30 ) - ShmFw::Vector4<int> ( 10, 10, 50, 50 );
        EXPECT_EQ ( true, varComputed == varComputed );
	}
    {
        ShmFw::Vector4<double> varResult ( 30, 0, 80.2, 60.2 ), varComputed;
        varComputed = ShmFw::Vector4<double> ( 20, 10, 30, 20 ) + ShmFw::Vector4<double> ( 10, -10, 50.2, 40.2 );
        EXPECT_EQ ( true, varComputed == varComputed );
	}
}

TEST_F ( ObjectTest, operationsDotVector4 ) {
    {
        int varComputed = ShmFw::Vector4<int> ( 20, 10, 30, 30 )  * ShmFw::Vector4<int> ( 10, 10, 50, 30 );
        EXPECT_EQ ( 2700, varComputed );
	}
    {
        float varComputed = ShmFw::Vector4<float> ( 20, 10, 30, 33 )  * ShmFw::Vector4<float> ( 10, 10, 50.1, 332 );
        EXPECT_EQ ( 12759, varComputed );
	}
}

TEST_F ( ObjectTest, operationsCrossVector4 ) {
  /*
    {
        ShmFw::Vector4<double> varResult ( 200, -700, 100 ), varComputed;
        varComputed = ShmFw::Vector4<double> ( 20, 10, 30 ) ^ (ShmFw::Vector4<double> ( 10, 10, 50 ));
        EXPECT_EQ ( true, varComputed == varResult);
	}
	*/
}
TEST_F ( ObjectTest, operationsScalarVector4 ) {
    {
        ShmFw::Vector4<int> varResult ( 10, 0, 20, 20 ), varComputed;
        varComputed = ShmFw::Vector4<int> ( 20, 10, 30, 30 ) - ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<int> ( 20, 10, 30, 30 ) - ( float ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<int> ( 20, 10, 30, 30 ) - ( double ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
    }  
    {
        ShmFw::Vector4<float> varResult ( 10.1, 0.2, 20.3, 20.3 ), varComputed;
        varComputed = ShmFw::Vector4<float> ( 20.1, 10.2, 30.3, 30.3 ) - ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<float> ( 20.1, 10.2, 30.3, 30.3 ) - ( float ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<float> ( 20.1, 10.2, 30.3, 30.3 ) - ( double ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
    }   
    {
        ShmFw::Vector4<double> varResult ( 10.1, 0.2, 20.3, 20.3 ), varComputed;
        varComputed = ShmFw::Vector4<double> ( 20.1, 10.2, 30.3, 30.3 ) - ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<double> ( 20.1, 10.2, 30.3, 30.3 ) - ( float ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<double> ( 20.1, 10.2, 30.3, 30.3 ) - ( double ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector4<int> varResult ( 20, 20, 20, 20 ), varComputed;
        varComputed = ShmFw::Vector4<int> ( 10, 10, 10, 10 ) + ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<int> ( 10, 10, 10, 10 ) + ( float ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<int> ( 10, 10, 10, 10 ) + ( double ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector4<float> varResult ( 20.1, 20.2, 20.3, 20.3 ), varComputed;
        varComputed = ShmFw::Vector4<float> ( 10.1, 10.2, 10.3, 10.3 ) + ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<float> ( 10.1, 10.2, 10.3, 10.3 ) + ( float ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<float> ( 10.1, 10.2, 10.3, 10.3 ) + ( double ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector4<double> varResult ( 20.1, 20.2, 20.3, 20.3 ), varComputed;
        varComputed = ShmFw::Vector4<double> ( 10.1, 10.2, 10.3, 10.3 ) + ( int ) 10;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<double> ( 10.1, 10.2, 10.3, 10.3 ) + ( float ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<double> ( 10.1, 10.2, 10.3, 10.3 ) + ( double ) 10.0;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector4<int> varResult ( 30, 45, 60, 60 ), varComputed;
        varComputed = ShmFw::Vector4<int> ( 6, 9, 12, 12 ) * ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<int> ( 6, 9, 12, 12 ) * ( float ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<int> ( 6, 9, 12, 12 ) * ( double ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector4<float> varResult ( 30, 45, 60, 60 ), varComputed;
        varComputed = ShmFw::Vector4<float> ( 6, 9, 12, 12 ) * ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<float> ( 20, 30, 40, 40 ) * ( float ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<float> ( 20, 30, 40, 40 ) * ( double ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector4<double> varResult ( 30, 45, 60, 60 ), varComputed;
        varComputed = ShmFw::Vector4<double> ( 6, 9, 12, 12 ) * ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<double> ( 20, 30, 40, 40 ) * ( float ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<double> ( 20, 30, 40, 40  ) * ( double ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
    {
        ShmFw::Vector4<int> varResult ( 6, 9, 12, 12 ), varComputed;
        varComputed = ShmFw::Vector4<int> ( 30, 45, 60, 60 ) / ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<int> ( 30, 45, 60, 60 ) / ( float ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<int> ( 30, 45, 60, 60 ) / ( double ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector4<float> varResult ( 6, 9, 12, 12 ), varComputed;
        varComputed = ShmFw::Vector4<float> ( 30, 45, 60, 60 ) / ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<float> ( 9, 13.5, 18, 18 ) / ( float ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<float> ( 9, 13.5, 18, 18 ) / ( double ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
    } 
    {
        ShmFw::Vector4<double> varResult ( 6, 9, 12, 12 ), varComputed;
        varComputed = ShmFw::Vector4<double> ( 30, 45, 60, 60 ) / ( int ) 5;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<double> ( 9, 13.5, 18, 18 ) / ( float ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
        varComputed = ShmFw::Vector4<double> ( 9, 13.5, 18, 18 ) / ( double ) 1.5 ;
        EXPECT_EQ ( true, varComputed == varComputed );
    }
}
TEST_F ( ObjectTest, equalVector4Int ) {
    ShmFw::Vector4<int> a ( 0, 0, 0, 0 );
    ShmFw::Vector4<int> b ( 2, 0, 0, 0 );
    bool similar = a.equal ( b, 3 );
    EXPECT_TRUE ( similar ) << "Should be equal";
    similar = a.equal ( b, 1 );
    EXPECT_FALSE ( similar ) << "Should no be equal";
}


TEST_F ( ObjectTest, equalVector4Double ) {
    ShmFw::Vector4<double> a ( 0, 0, 0, 0 );
    ShmFw::Vector4<double> b ( 1, 1, 0, 0 );
    bool similar = a.equal ( b, sqrt(2)+0.1 );
    EXPECT_TRUE ( similar ) << "Should be equal";
    similar = a.equal ( b, sqrt(2) );
    EXPECT_FALSE ( similar ) << "Should no be equal";
}

TEST_F ( ObjectTest, copyToFromVector4 ) {
    ShmFw::Quaternion a(rand(), rand(), rand(), rand()), c;
    ShmFw::Vector4<double> b, d;
    b.copyFrom(a);
    b.copyTo(d);
    c.copyFrom(d);
    c.copyTo(d);
    std::cout << "a: " << a << ", d: " << d << std::endl;
    EXPECT_EQ ( a, c );
    EXPECT_EQ ( b, d );
}
}  // namespace

