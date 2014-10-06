#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <shmfw/serialization/interprocess_vector.hpp>

typedef boost::interprocess::allocator<int, boost::interprocess::managed_shared_memory::segment_manager> ShmemAllocator;
typedef boost::interprocess::vector<int, ShmemAllocator> MyVector;

void write ( const std::string &filename, const MyVector &src ) {
    std::ofstream ofs ( filename.c_str() );
    assert ( ofs.good() );
    boost::archive::xml_oarchive xml ( ofs );
    xml << boost::serialization::make_nvp ( "Vec", src );
}

void read ( const std::string &filename, MyVector &des) {
    std::ifstream ifs ( filename.c_str() );
    assert ( ifs.good() );
    boost::archive::xml_iarchive xml ( ifs );
    xml >> boost::serialization::make_nvp ( "Vec", des );
}


int main () {
    //Remove shared memory on construction and destruction
    struct shm_remove {
        shm_remove() {
            boost::interprocess::shared_memory_object::remove ( "MySharedMemory" );
        }
        ~shm_remove() {
            boost::interprocess::shared_memory_object::remove ( "MySharedMemory" );
        }
    } remover;

    //A managed shared memory where we can construct objects
    //associated with a c-string
    boost::interprocess::managed_shared_memory segment ( boost::interprocess::create_only,
                                    "MySharedMemory",  //segment name
                                    65536 );


    int initVal[]        = {rand(), rand(), rand(), rand(), rand(), rand(), rand() };
    const int *begVal    = initVal;
    const int *endVal    = initVal + sizeof ( initVal ) /sizeof ( initVal[0] );

    //Initialize the STL-like allocator
    const ShmemAllocator alloc_inst ( segment.get_segment_manager() );
    const ShmemAllocator alloc_inst2 ( segment.get_segment_manager() );

    //Construct the vector in the shared memory segment with the STL-like allocator
    //from a range of iterators
    MyVector *vector_src =
        segment.construct<MyVector>
        ( "MyVectorSrc" ) /*object name*/
        ( begVal     /*first ctor parameter*/,
          endVal     /*second ctor parameter*/,
          alloc_inst /*third ctor parameter*/ );

    MyVector *vector_des =
        segment.construct<MyVector>
        ( "MyVectorDes" ) ( alloc_inst );

    //Use vector as your want
    std::sort ( vector_src->rbegin(), vector_src->rend() );


    write ( "/tmp/vector.xml", *vector_src );
    read ( "/tmp/vector.xml", *vector_des );

    if ( vector_src->size() == vector_des->size() ) {
        for ( size_t i = 0; i < vector_des->size(); i++ ) {
            std::cout << i << ": " << vector_src->at ( i ) << " = " << vector_des->at ( i ) << std::endl;
        }
    } else {
        std::cout << "Something went wrong!" << std::endl;
    }



    segment.destroy<MyVector> ( "MyVectorSrc" );
    segment.destroy<MyVector> ( "MyVectorDes" );
    return 0;
}
