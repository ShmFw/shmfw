
#include "gtest/gtest.h"
#include <fstream>
#include "shmfw/objects/matrix3x3.h"
#include "shmfw/objects/plane.h"
#include "shmfw/objects/vector2.h"
#include "shmfw/objects/vector3.h"
#include "shmfw/objects/vector4.h"
#include "shmfw/objects/vec.h"
#include "shmfw/objects/point.h"
#include "shmfw/objects/point2d.h"
#include "shmfw/objects/twist.h"
#include "shmfw/objects/quaternion.h"
#include "shmfw/serialization/io_file.h"

#ifndef SHARED_MEM_UNITTEST_OBJECTS_H
#define SHARED_MEM_UNITTEST_OBJECTS_H

namespace ShmFwTest {




// The fixture for testing class Foo.
class ObjectTest : public ::testing::Test {
protected:
    // You can remove any or all of the following functions if its body
    // is empty.

    ObjectTest()
        : shmSegmentName_ ( "ShmTestSegment" )
        , shmSegmentSize_ ( 65536 ) {
        // You can do set-up work for each test here.
        srand ( time ( NULL ) );
    }

    virtual ~ObjectTest() {
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

}  // namespace
#endif //SHARED_MEM_UNITTEST_OBJECTS_H
