
#include <mgl2/qt.h>
#include "mathgl_prepare_data.h"

int sample ( mglGraph *gr ) {
    mglData a;
    mgls_prepare2d ( &a );
    gr->SetOrigin(-1,1,-1);
    gr->Title ( "Mesh plot" );
    gr->Rotate ( 50,60 );
    gr->Box();
    gr->Mesh ( a );
    gr->SetRanges(-10,10,-1,1);
    gr->Axis(); gr->Grid();
    return 0;
}
//-----------------------------------------------------
int main ( int argc,char **argv ) {
    mglQT gr ( sample,"MathGL examples" );
    return gr.Run();
}
