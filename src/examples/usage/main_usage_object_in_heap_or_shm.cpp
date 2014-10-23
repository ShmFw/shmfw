#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/range/algorithm.hpp>
#include <iostream>
#include <cassert>

namespace bip = boost::interprocess;
template <typename T> 
    using BoundShmemAllocator = bip::allocator<T, bip::managed_shared_memory::segment_manager>;

///////////////////////////////////////////////////////////////
// Your MyStruct, templatized for an Allocator class template

template <template<typename...> class Allocator>
class MyStruct {
public:
    bip::vector<int,    Allocator<int>    > ints;
    bip::vector<double, Allocator<double> > doubles;

    MyStruct(const Allocator<void>& void_alloc = {})
        : ints(void_alloc),
          doubles(void_alloc)
    {}
};

// Variant to use on the heap:
using HeapStruct  = MyStruct<std::allocator>;
// Variant to use in shared memory:
using ShmemStruct = MyStruct<BoundShmemAllocator>;

//
///////////////////////////////////////////////////////////////

int main() {
    srand(time(NULL));

    // You can have something like this working: 
    HeapStruct x; // and also the shm stuff below
    std::generate_n(std::back_inserter(x.ints),    20, &std::rand);
    std::generate_n(std::back_inserter(x.doubles), 20, &std::rand);

    // A managed shared memory where we can construct objects
    bip::managed_shared_memory segment = bip::managed_shared_memory(bip::open_or_create, "MySharedMemory", 65536);
    BoundShmemAllocator<int> const shmem_alloc(segment.get_segment_manager());

    auto src = segment.find_or_construct<ShmemStruct>("MyStruct")(shmem_alloc);
    src->ints.insert(src->ints.end(),       x.ints.begin(),    x.ints.end());
    src->doubles.insert(src->doubles.end(), x.doubles.begin(), x.doubles.end());

    auto des = segment.find_or_construct<ShmemStruct>("MyStruct")(shmem_alloc);

    std::cout << "-------------------------";
    boost::copy(src->ints, std::ostream_iterator<int>(std::cout << "\nsrc ints: ", "; "));
    boost::copy(des->ints, std::ostream_iterator<int>(std::cout << "\ndes ints: ", "; "));
    std::cout << "\n-------------------------";
    boost::copy(src->doubles, std::ostream_iterator<double>(std::cout << "\nsrc doubles: ", "; "));
    boost::copy(des->doubles, std::ostream_iterator<double>(std::cout << "\ndes doubles: ", "; "));

    assert(src->ints.size()    == des->ints.size());
    assert(src->doubles.size() == des->doubles.size());
    assert(boost::mismatch(src->ints,    des->ints)    == std::make_pair(src->ints.end(),    des->ints.end()));
    assert(boost::mismatch(src->doubles, des->doubles) == std::make_pair(src->doubles.end(), des->doubles.end()));

    segment.destroy<ShmemStruct>("MyStruct");
}