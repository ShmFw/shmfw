
#include "unittest_objects.h"
#include <time.h>

int main ( int argc, char **argv ) {
    /* initialize random seed: */
    testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}

