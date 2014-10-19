
#include "shmfw/variable.h"
#include "shmfw/allocator.h"
#include "shmfw/objects/points.h"
#include "shmfw/objects/parameterentry.h"
#include <boost/thread.hpp>
#include "gtest/gtest.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <shmfw/serialization/variable.h>
#include <shmfw/serialization/vector.h>
#include <shmfw/serialization/deque.h>
#include <shmfw/serialization/io_file.h>
#include <time.h>


template<typename T>
void triggerItHasChanged ( ShmFw::HandlerPtr &shmHdl, std::string &name, int ms ) {
    ShmFw::Var<T> shmVar ( name, shmHdl, 1 );
    usleep ( ms*1000 );
    shmVar.itHasChanged();
}

template<typename T>
void triggerLock ( ShmFw::HandlerPtr &shmHdl, std::string &name, int ms ) {
    ShmFw::Var<T> shmVar ( name, shmHdl, 1 );
    usleep ( ms*1000 );
    shmVar.lock();
}

template<typename T>
void triggerUnLock ( ShmFw::HandlerPtr &shmHdl, std::string &name, int ms ) {
    ShmFw::Var<T> shmVar ( name, shmHdl, 1 );
    usleep ( ms*1000 );
    shmVar.unlock();
}

namespace {


// The fixture for testing class Foo.
class ShmTest : public ::testing::Test {
protected:
    // You can remove any or all of the following functions if its body
    // is empty.

    ShmTest()
        : shmSegmentName_ ( "ShmTestSegment" )
        , shmSegmentSize_ ( 65536 ) {
        // You can do set-up work for each test here.
    }

    virtual ~ShmTest() {
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
TEST_F ( ShmTest, TestParameterEntry ) {
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<ShmFw::ParameterEntry<int> > a ( "a", shmHdl );
    a() = 10;
    EXPECT_EQ ( a(), 10 );
    ShmFw::Var<ShmFw::ParameterEntry<int> > b ( "a", shmHdl );
    EXPECT_EQ ( b(), 10 );
    int v = 20, min = -10, max = 200, step = 3;
    b() = ShmFw::ParameterEntry<int> ( v, min, max, step );
    EXPECT_EQ ( b(), v );
    EXPECT_EQ ( b().max(), max );
    EXPECT_EQ ( b().min(), min );
    EXPECT_EQ ( b().step_size(), step );
    EXPECT_TRUE ( b().valid() );
    EXPECT_TRUE ( b().enable() );
    a().enable ( false );
    EXPECT_FALSE ( b().enable() );
    EXPECT_TRUE ( b().valid() );
    a().enable ( true );
    b() = min-1;
    EXPECT_FALSE ( b().valid() );
    b() = min;
    EXPECT_TRUE ( b().valid() );
    b() = max+1;
    EXPECT_FALSE ( b().valid() );
    b() = max;
    EXPECT_TRUE ( b().valid() );
    b() = v;
    EXPECT_TRUE ( b().valid() );
    bool compareOperators;
    compareOperators = b() < ( v+1 );
    EXPECT_TRUE ( compareOperators );
    compareOperators = b() > ( v+1 );
    EXPECT_FALSE ( compareOperators );
    compareOperators = b() < ( v-1 );
    EXPECT_FALSE ( compareOperators );
    compareOperators = b() > ( v-1 );
    EXPECT_TRUE ( compareOperators );
    b() = max;
    b().increase();
    EXPECT_EQ ( b(), max );
    b() = min;
    b().decrease();
    EXPECT_EQ ( b(), min );
    b() = v;
    b().increase();
    EXPECT_EQ ( b(), v+step );
    b().decrease();
    EXPECT_EQ ( b(), v );
    /*
    ShmFw::Var<ShmFw::ParameterEntry<double> > c ( "c", shmHdl);
    ShmFw::Var<ShmFw::ParameterEntry<double> > d ( "a", shmHdl);
    try{
    c() = d();
    } catch (int e){
    int exeptionID = ShmFw::ParameterEntryHeader::EXEPTION_TYPE_ASSIGNMENT;
    EXPECT_EQ (e, exeptionID);
    }
    */
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, AllocPoints ) {
  
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Alloc<ShmFw::Points> a ( "var0", shmHdl ) ;
    a->points.push_back(ShmFw::Point(rand_01(), rand_01(), rand_01()));
    a->points.push_back(ShmFw::Point(rand_01(), rand_01(), rand_01()));
    a().frame = "world";
    ShmFw::Alloc<ShmFw::Points> b ( "var0", shmHdl ) ;
    EXPECT_EQ (a(), b());
    ShmFw::Alloc<ShmFw::Points> c ( "var1", shmHdl ) ;
    c().copyFrom(b());
    EXPECT_EQ (a(), c());
    a->points[0].x = 3;
    EXPECT_TRUE (a() == b());
    EXPECT_FALSE (a() == c());
    // std::cout << "a: " << a() << "c: " << c();
    shmHdl->removeSegment();

}
TEST_F ( ShmTest, TestSharedLockPtr ) {

    /**
     * @ToDo
      std::string name("a");
      ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
      ShmFw::Var<int> a ( name, shmHdl, 1 );
      ShmFw::ScopedLockPtr lock;
      boost::thread t1 ( triggerItHasChanged<double>, shmHdl, name, 1000 );
      a.wait ( lock );
      bool isLocked = a.locked();
      EXPECT_TRUE ( isLocked ) << "it should be locked";
      lock.reset();
      isLocked = a.locked();
      EXPECT_FALSE ( isLocked ) << "it not should be locked";
      shmHdl->removeSegment();
      */
}

TEST_F ( ShmTest, TestClasses ) {

    std::string nameA ( "myVarA" );
    std::string nameB ( "myVarB" );
    std::string nameC ( "myVarC" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<int> a ( nameA, shmHdl, 1 );
    ShmFw::Var<double> b ( nameB, shmHdl, 1 );
    ShmFw::Var<double> c ( nameC, shmHdl, 1 );
    //std::cout << a.type_name() << std::endl;
    EXPECT_FALSE ( a.isType<ShmFw::Var<double> >() );
    EXPECT_TRUE ( a.isType<ShmFw::Var<int> >() );
    shmHdl->removeSegment();
}


TEST_F ( ShmTest, TestTypeName ) {

    std::string nameA ( "myVarA" );
    std::string nameB ( "myVarB" );
    std::string nameC ( "myVarC" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<int> a ( nameA, shmHdl, 1 );
    ShmFw::Var<double> b ( nameB, shmHdl, 1 );
    ShmFw::Var<double> c ( nameC, shmHdl, 1 );
    //std::cout << a.type_name() << std::endl;
    EXPECT_FALSE ( a.isType<ShmFw::Var<double> >() );
    EXPECT_TRUE ( a.isType<ShmFw::Var<int> >() );
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, TestSimpleAssignment ) {

    std::string name ( "myVar" );
    srand ( time ( NULL ) );
    double v = rand() % 100;
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<double> a ( name, shmHdl, 1 );
    a = v;
    ShmFw::Var<double> b ( name, shmHdl, 1 );
    EXPECT_EQ ( 0, a()-v );
    EXPECT_EQ ( 0, b()-v );
    EXPECT_EQ ( 0, a()-b() );
    a.set ( v+1 );
    EXPECT_EQ ( v+1, a() );
    double v2;
    b.get ( v2 );
    EXPECT_EQ ( v+1, v2 );
    EXPECT_EQ ( v2, b[0] );
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, TestLocks ) {
    std::string name ( "myVar" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<double> a ( name, shmHdl, 1 );
    a.lock();
    boost::thread t1 ( triggerUnLock<double>, shmHdl, name, 5 );
    usleep ( 1000 );
    EXPECT_FALSE ( a.timed_lock ( 1 ) );
    EXPECT_TRUE ( a.timed_lock ( 10 ) );
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, TestChanged ) {
    std::string name ( "myVar" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<double> a ( name, shmHdl, 1 );
    double v = 3;
    a = 3;
    EXPECT_EQ ( v, a() );
    EXPECT_FALSE ( a.hasChanged() );
    ShmFw::Var<double> b ( name, shmHdl, 1 );
    b = v + 1;
    b.itHasChanged();
    EXPECT_TRUE ( a.hasChanged ( ) );
    EXPECT_TRUE ( a() !=v );
    a.dataProcessed();
    EXPECT_FALSE ( a.hasChanged ( ) );
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, TestSignals ) {
    std::string name ( "myVar" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<double> a ( name, shmHdl, 1 );
    boost::thread t1 ( triggerItHasChanged<double>, shmHdl, name, 5 );
    EXPECT_FALSE ( a.timed_wait ( 1 ) );
    EXPECT_TRUE ( a.timed_wait ( 10 ) );
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, StringVector ) {
    std::string name ( "var0" );
    std::string entry0 ( "Hello" );
    std::string entry1 ( " " );
    std::string entry2 ( "World" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::VectorStr a (name, shmHdl );
    a.clear();
    a.push_back(entry0);
    a.push_back(entry1);
    a.push_back(entry2);
    ShmFw::VectorStr b (name, shmHdl );
    EXPECT_TRUE ( (b[0].compare (entry0.c_str()) == 0) );
    EXPECT_TRUE ( (b[1].compare (entry1.c_str()) == 0) );
    EXPECT_FALSE ( (b[2].compare (entry0.c_str()) == 0) );  
    shmHdl->removeSegment();
}


TEST_F ( ShmTest, TestSerializeXML ) {
    std::string filename ( "unittest.txt" );
    std::string name1 ( "myVar1" );
    std::string name2 ( "myVar2" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<double> a ( name1, shmHdl, 1 );
    unsigned int r = rand();
    a.set ( r );
    ShmFw::write ( filename,  a, ShmFw::FORMAT_XML );
    ShmFw::Var<double> b ( name2, shmHdl, 1 );
    ShmFw::read ( filename,  b, ShmFw::FORMAT_XML );
    EXPECT_TRUE ( a() == b() );
    EXPECT_TRUE ( a.timestampShm() == b.timestampShm() );
    EXPECT_TRUE ( a.size() == b.size() );
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, TestSerializeTXT ) {
    std::string filename ( "unittest.txt" );
    std::string name1 ( "myVar1" );
    std::string name2 ( "myVar2" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<double> a ( name1, shmHdl, 1 );
    ShmFw::Var<double> b ( name2, shmHdl, 1 );
    unsigned int r = rand();
    a.set ( r );
    {
        std::ofstream ofs ( filename.c_str() );
        boost::archive::text_oarchive oa ( ofs );
        oa << a;
    }
    {
        std::ifstream ifs ( filename.c_str() );
        boost::archive::text_iarchive ia ( ifs );
        ia >> b;
    }
    EXPECT_TRUE ( a() == b() );
    EXPECT_TRUE ( a.timestampShm() == b.timestampShm() );
    EXPECT_TRUE ( a.size() == b.size() );
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, TestSerializeBinary ) {
    std::string filename ( "unittest.bin" );
    std::string name1 ( "myVar1" );
    std::string name2 ( "myVar2" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<double> a ( name1, shmHdl, 1 );
    ShmFw::Var<double> b ( name2, shmHdl, 1 );
    unsigned int r = rand();
    a.set ( r );
    {
        std::ofstream ofs ( filename.c_str() );
        boost::archive::binary_oarchive oa ( ofs );
        oa << a;
    }
    {
        std::ifstream ifs ( filename.c_str() );
        boost::archive::binary_iarchive ia ( ifs );
        ia >> b;
    }
    EXPECT_TRUE ( a() == b() );
    EXPECT_TRUE ( a.timestampShm() == b.timestampShm() );
    EXPECT_TRUE ( a.size() == b.size() );
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, TestSerializeVectorXML ) {
    std::string filename ( "points.xml" );
    std::string name1 ( "myVector1" );
    std::string name2 ( "myVector2" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Vector<double> a ( name1, shmHdl );
    ShmFw::Vector<double> b ( name2, shmHdl );
    a().clear();
    for ( size_t i = 0; i < 5; i++ ) a.push_back ( rand() );

    ShmFw::write ( filename, a, ShmFw::FORMAT_XML );
    ShmFw::read ( filename, b, ShmFw::FORMAT_XML );
    EXPECT_TRUE ( a() == b() );
    EXPECT_TRUE ( a.timestampShm() == b.timestampShm() );
    EXPECT_TRUE ( a.size() == b.size() );
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, TestSerializeDequeXML ) {
    std::string filename ( "pointsDeque.xml" );
    std::string name1 ( "myDeque1" );
    std::string name2 ( "myDeque2" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Deque<int> a ( name1, shmHdl );
    ShmFw::Deque<int> b ( name2, shmHdl );
    a().clear();
    for ( size_t i = 0; i < 5; i++ ) a.push_back ( rand() );
    ShmFw::write ( filename, a, ShmFw::FORMAT_XML );
    ShmFw::read ( filename, b, ShmFw::FORMAT_XML );
    EXPECT_TRUE ( a() == b() );
    EXPECT_TRUE ( a.timestampShm() == b.timestampShm() );
    EXPECT_TRUE ( a.size() == b.size() );
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, TestTypeNameShm ) {
    std::string name0 ( "v0" );
    std::string name1 ( "v1" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<int> a ( name0, shmHdl );
    ShmFw::Var<int> b ( name1, shmHdl );
    bool equal = a.isType<ShmFw::Var<int> >();
    EXPECT_TRUE (equal );
    shmHdl->removeSegment();
}


TEST_F ( ShmTest, TestInfoTextShm ) {
    std::string name ( "v0" );
    std::string infoA ( "hallo world" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<int> a ( name, shmHdl );
    a.info_text(infoA);
    ShmFw::Var<int> b ( name, shmHdl );
    std::string infoB = b.info_text();
    int idx = infoB.compare(infoA);
    EXPECT_EQ ( idx, 0 );
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, TestInfoTextSerialization ) {
    std::string filename ( "v0.xml" );
    std::string name0 ( "v0" );
    std::string name1 ( "v1" );
    std::string infoA ( "hallo world" );
    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shmSegmentName_, shmSegmentSize_ );
    ShmFw::Var<int> a ( name0, shmHdl );
    ShmFw::Var<int> b ( name1, shmHdl );
    a.info_text(infoA);
    ShmFw::write ( filename, a, ShmFw::FORMAT_XML );
    ShmFw::read ( filename, b, ShmFw::FORMAT_XML );
    std::string infoB = b.info_text();
    int idx = infoB.compare(infoA);
    EXPECT_EQ ( idx, 0 );
    shmHdl->removeSegment();
}

TEST_F ( ShmTest, file_postix ) {
    EXPECT_TRUE ( ShmFw::FORMAT_NA == ShmFw::file_postix ( "" ) );
    EXPECT_TRUE ( ShmFw::FORMAT_NA == ShmFw::file_postix ( "." ) );
    EXPECT_TRUE ( ShmFw::FORMAT_NA == ShmFw::file_postix ( ".file" ) );
    EXPECT_TRUE ( ShmFw::FORMAT_NA == ShmFw::file_postix ( "file." ) );
    EXPECT_TRUE ( ShmFw::FORMAT_XML == ShmFw::file_postix ( "file.xml" ) );
    EXPECT_TRUE ( ShmFw::FORMAT_XML == ShmFw::file_postix ( "file.XML" ) );
    EXPECT_TRUE ( ShmFw::FORMAT_BIN == ShmFw::file_postix ( "file.bin" ) );
    EXPECT_TRUE ( ShmFw::FORMAT_BIN == ShmFw::file_postix ( "file.BIN" ) );
    EXPECT_TRUE ( ShmFw::FORMAT_TXT == ShmFw::file_postix ( "file.TxT" ) );
    EXPECT_TRUE ( ShmFw::FORMAT_TXT == ShmFw::file_postix ( "file.txt" ) );
}

}  // namespace



int main ( int argc, char **argv ) {
    srand ( time ( NULL ) );
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}

