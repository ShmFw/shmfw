

#include <iostream>
#include <string.h>
#include <vector>
#include <stdio.h>
#include <shmfw/serialization/variable.h>
#include <shmfw/serialization/io_file.h>

int main() {

    boost::shared_ptr<ShmFw::Handler> shmHdl = ShmFw::Handler::create ( ShmFw::DEFAULT_SEGMENT_NAME(), ShmFw::DEFAULT_SEGMENT_SIZE() );



    std::string filename ( "/tmp/variable.xml" );
    std::cout << "-------------- Variable read/write --------------" << std::endl;
    std::cout << "to -> " << filename << std::endl;

    ShmFw::Var<int> a ( "variableA", shmHdl );
    ShmFw::Var<int> b ( "variableB", shmHdl );


    a() = rand();

    ShmFw::write ( filename, a, ShmFw::FORMAT_XML );
    ShmFw::read ( filename, b, ShmFw::FORMAT_XML );

    std::cout  << a << " = " << b << std::endl;

    return 0;
}
