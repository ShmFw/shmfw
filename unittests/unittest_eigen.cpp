#include <iostream>
#include "gtest/gtest.h"
#include <Eigen/Dense>
#include "shmfw/variable.h"
#include "shmfw/serialization/eigen.h"
#include "shmfw/serialization/io_file.h"

namespace {

// The fixture for testing class Foo.
class EigenTest : public ::testing::Test {
protected:
    // You can remove any or all of the following functions if its body
    // is empty.

    EigenTest()
        : shmSegmentName_ ( "ShmTestSegment" )
        , shmSegmentSize_ ( 65536 ) {
        srand ( time ( NULL ) );
        // You can do set-up work for each test here.
    }

    virtual ~EigenTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp() {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    virtual void TearDown() {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    std::string shmSegmentName_;
    int shmSegmentSize_;
    double rand_01() {
        return ( double ) rand() / RAND_MAX;
    }
    double randf ( double fmin, double fmax ) {
        return rand_01() * ( fmax - fmin ) + fmin;
    }
    double randAngle() {
        return rand_01() * 2 * M_PI - M_PI;
    }
    double randf() {
        return randf ( -10, 10 );
    }
};

TEST_F ( EigenTest, ShmVector3d ) {

    std::string name ( "var0" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<Eigen::Vector3d> a ( name,shmHdl );
    ShmFw::Var<Eigen::Vector3d> b ( name,shmHdl );
    *a << rand_01(), rand_01(), rand_01();
    EXPECT_EQ ( *a, *b );
    shmHdl->removeSegment();
}
TEST_F ( EigenTest, Matrix3d ) {

    std::string name ( "var0" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<Eigen::Matrix3d> a ( name,shmHdl );
    ShmFw::Var<Eigen::Matrix3d> b ( name,shmHdl );
    *a = Eigen::Matrix3d::Random();
    EXPECT_EQ ( *a, *b );
    shmHdl->removeSegment();
}
TEST_F ( EigenTest, serializeVector3dXML ) {
    std::string filename ( "/tmp/Vector3d.xml" );
    Eigen::Vector3d a (rand_01(), rand_01(), rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( EigenTest, serializeMatrix3dXML ) {
    std::string filename ( "/tmp/Matrix3d.xml" );
    Eigen::Matrix3d a = Eigen::Matrix3d::Random(), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( EigenTest, serializeVector3dBin ) {
    std::string filename ( "/tmp/Vector3d.bin" );
    Eigen::Vector3d a (rand_01(), rand_01(), rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( EigenTest, serializeMatrix3dBin ) {
    std::string filename ( "/tmp/Matrix3d.bin" );
    Eigen::Matrix3d a = Eigen::Matrix3d::Random(), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( EigenTest, serializeVector3dTxt ) {
    std::string filename ( "/tmp/Vector3d.txt" );
    Eigen::Vector3d a (rand_01(), rand_01(), rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( EigenTest, serializeMatrix3dTxt ) {
    std::string filename ( "/tmp/Matrix3d.txt" );
    Eigen::Matrix3d a = Eigen::Matrix3d::Random(), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( EigenTest, serializeVectorXdXML ) {
    std::string filename ( "/tmp/VectorXd.xml" );
    Eigen::VectorXd a (3), b;
    a << rand_01(), rand_01(), rand_01();;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( EigenTest, serializeMatrixXdXML ) {
    std::string filename ( "/tmp/MatrixXd.xml" );
    Eigen::MatrixXd a = Eigen::MatrixXd::Random ( 3,3 ), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( EigenTest, serializeVectorXdBin ) {
    std::string filename ( "/tmp/VectorXd.bin" );
    Eigen::VectorXd a (3), b;
    a << rand_01(), rand_01(), rand_01();;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( EigenTest, serializeMatrixXdBin ) {
    std::string filename ( "/tmp/MatrixXd.bin" );
    Eigen::MatrixXd a = Eigen::MatrixXd::Random ( 3,3 ), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( EigenTest, serializeVectorXdTxt ) {
    std::string filename ( "/tmp/VectorXd.txt" );
    Eigen::VectorXd a (3), b;
    a << rand_01(), rand_01(), rand_01();;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( EigenTest, serializeMatrixXdTxt ) {
    std::string filename ( "/tmp/MatrixXd.txt" );
    Eigen::MatrixXd a = Eigen::MatrixXd::Random ( 3,3 ), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}


}  // namespace



int main ( int argc, char **argv ) {
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}

