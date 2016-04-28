
#include "shmfw/allocator.h"
#include "shmfw/objects/opencv.h"
#include "shmfw/objects/image.h"
#include "shmfw/serialization/vector.h"
#include "shmfw/serialization/opencv.h"
#include "shmfw/serialization/io_file.h"
#include "gtest/gtest.h"
#include "opencv2/highgui/highgui.hpp"
#include <time.h>

namespace {
  
// The fixture for testing class Foo.
class OpenCVTest : public ::testing::Test {
protected:
    // You can remove any or all of the following functions if its body
    // is empty.

    OpenCVTest()
        : shmSegmentName_ ( "ShmTestSegment" )
        , shmSegmentSize_ ( 16777216) {
	srand ( time ( NULL ) );
        // You can do set-up work for each test here.
    }

    virtual ~OpenCVTest() {
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

TEST_F ( OpenCVTest, serializeCVPoint ) {
    std::string filename ( "/tmp/cvPoint.xml" );
    cv::Point a(rand(),rand()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}

TEST_F ( OpenCVTest, serializeCVPoint2f ) {
    std::string filename ( "/tmp/cvPoint2f.xml" );
    cv::Point2f a(rand_01(),rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}


TEST_F ( OpenCVTest, serializeCVPoint2d ) {
    std::string filename ( "/tmp/cvPoint2d.xml" );
    cv::Point2d a(rand_01(),rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( OpenCVTest, serializeCVRect ) {
    std::string filename ( "/tmp/cvRect.xml" );
    cv::Rect a(rand(),rand(), rand(),rand()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( OpenCVTest, serializeCVRectf ) {
    std::string filename ( "/tmp/cvRectf.xml" );
    cv::Rect_<float> a(rand_01(),rand_01(), rand_01(),rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}
TEST_F ( OpenCVTest, serializeCVRectd ) {
    std::string filename ( "/tmp/cvRectd.xml" );
    cv::Rect_<double> a(rand_01(),rand_01(), rand_01(),rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}

TEST_F ( OpenCVTest, serializeCVSize ) {
    std::string filename ( "/tmp/cvSize.xml" );
    cv::Size a(rand_01(),rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}

TEST_F ( OpenCVTest, serializeCVSizef ) {
    std::string filename ( "/tmp/cvSizef.xml" );
    cv::Size_<float> a(rand_01(),rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}

TEST_F ( OpenCVTest, serializeCVSized ) {
    std::string filename ( "/tmp/cvSized.xml" );
    cv::Size_<double> a(rand_01(),rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}


TEST_F ( OpenCVTest, serializeCVRotatedRect ) {
    std::string filename ( "/tmp/cvRotatedRect.xml" );
    cv::RotatedRect a(cv::Point2f(rand_01(),rand_01()), cv::Size2f(rand_01(), rand_01()), rand_01()), b;
    ShmFw::write(filename, a);
    ShmFw::read (filename, b);
    EXPECT_EQ ( a, b );
}

TEST_F ( OpenCVTest, shmImage ) {
    if(getenv ( "SHMFW" ) == NULL) {
      std::cerr << "Set SHMFW: \"export SHMFW=$HOME/projects/shmfw\" for this test!"<< std::endl;
      return;
    }
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    std::string shm_name ( "image_maxi" );
    std::string image_filename = std::string ( getenv ( "SHMFW" ) ) + "/res/maxi.jpg";
    cv::Mat imgA = cv::imread(image_filename, CV_LOAD_IMAGE_COLOR);
    ShmFw::Alloc<ShmFw::ImageShm> a( shm_name, shmHdl );
    a->copyFrom(imgA, ShmFw::IMAGE_ENCODING_BGR8);
    ShmFw::Alloc<ShmFw::ImageShm> b( shm_name, shmHdl );
    cv::Mat imgShm;
    b->cvMat(imgShm);
    cv::putText(imgShm, "shmImage", cv::Point(0, imgShm.rows/2), cv::FONT_HERSHEY_COMPLEX,1, cv::Scalar(0xFF,0xFF,0xFF),1);
    cv::namedWindow ( image_filename, CV_WINDOW_AUTOSIZE );
    cv::imshow ( image_filename, imgShm );
    cv::waitKey(1000);
    cv::destroyWindow(image_filename);
    cv::waitKey(100);
    shmHdl->removeSegment();
}
TEST_F ( OpenCVTest, serializeImage ) {
    if(getenv ( "SHMFW" ) == NULL) {
      std::cerr << "Set SHMFW: \"export SHMFW=$HOME/projects/shmfw\" for this test!"<< std::endl;
      return;
    }
    std::string filename ( "image.xml" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    std::string shm_name ( "image_maxi" );
    std::string image_filename = std::string ( getenv ( "SHMFW" ) ) + "/res/maxi.jpg";
    cv::Mat imgA = cv::imread(image_filename, CV_LOAD_IMAGE_COLOR);
    ShmFw::Alloc<ShmFw::ImageShm> a( shm_name, shmHdl );
    ShmFw::Alloc<ShmFw::ImageShm> b( shm_name, shmHdl );
    a->copyFrom(imgA, ShmFw::IMAGE_ENCODING_BGR8);    
    ShmFw::write(filename, *a);
    ShmFw::read (filename, *b);
    EXPECT_EQ ( *a, *b );
    cv::Mat imgShm;
    b->cvMat(imgShm);
    cv::putText(imgShm, "serializeImage", cv::Point(0, imgShm.rows/2), cv::FONT_HERSHEY_COMPLEX,1, cv::Scalar(0xFF,0xFF,0xFF),1);
    cv::namedWindow ( filename, CV_WINDOW_AUTOSIZE );
    cv::imshow ( filename, imgShm );
    cv::waitKey(1000);
    cv::destroyWindow(filename);
    cv::waitKey(100);
    shmHdl->removeSegment();
}
}  // namespace



int main ( int argc, char **argv ) {
    srand ( time ( NULL ) );
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}

