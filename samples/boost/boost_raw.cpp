#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <shmfw/serialization/interprocess_vector.hpp>
#include <boost/interprocess/managed_heap_memory.hpp>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


using namespace boost::interprocess;
//Alias an STL-like allocator of ints that allocates ints from the segment
typedef allocator<int, managed_shared_memory::segment_manager>  ShmemAllocator;

//Alias a vector that uses the previous STL-like allocator
typedef allocator<void, managed_shared_memory::segment_manager >  void_allocator;

template<typename SegmentManager>
class MyStruct {
    typedef allocator<int, SegmentManager>  Allocator;
    typedef vector<int, Allocator> MyVector;
public:
    MyVector myVector;
    //Since void_allocator is convertible to any other allocator<T>, we can simplify
    //the initialization taking just one allocator for all inner containers.
    MyStruct ( const allocator<void, SegmentManager > &void_alloc )
        : myVector ( void_alloc )
    {}
};

typedef MyStruct<managed_shared_memory::segment_manager> ShmMyStruct;
typedef MyStruct<managed_heap_memory::segment_manager> HeapMyStruct;

int main () {
    //Remove shared memory on construction and destruction
    /*
    struct shm_remove {
          shm_remove() {
              shared_memory_object::remove ( "MySharedMemory" );
          }
          ~shm_remove() {
              shared_memory_object::remove ( "MySharedMemory" );
          }
      } remover;
      */

    managed_heap_memory heap_memory ( 1000 );
    HeapMyStruct  x ( heap_memory.get_segment_manager() );

    managed_shared_memory segment;
    //A managed shared memory where we can construct objects
    //associated with a c-string
    try {
        segment =  managed_shared_memory ( create_only, "MySharedMemory", 65536 );
    } catch ( ... ) {
        segment = managed_shared_memory ( open_only, "MySharedMemory" );
    }

    //Initialize the STL-like allocator
    const ShmemAllocator alloc_inst ( segment.get_segment_manager() );

    ShmMyStruct *myStruct_src =  segment.find_or_construct<ShmMyStruct> ( "MyStruct" ) ( alloc_inst );
    srand ( time ( NULL ) );
    myStruct_src->myVector.push_back ( rand() );

    ShmMyStruct *myStruct_des =  segment.find_or_construct<ShmMyStruct> ( "MyStruct" ) ( alloc_inst );

    for ( size_t i = 0; i < myStruct_src->myVector.size(); i++ ) {
        std::cout << i << ": " << myStruct_src->myVector[i] << " = " << myStruct_des->myVector[i] << std::endl;
        if ( myStruct_src->myVector[i] != myStruct_des->myVector[i] ) {
            std::cout << "Something went wrong!" << std::endl;
        }
    }


    //segment.destroy<MyVector> ( "MyVector" );
    return 0;
}
