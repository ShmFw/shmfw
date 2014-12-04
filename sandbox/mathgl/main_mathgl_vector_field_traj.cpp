
#include <mgl2/qt.h>
#include "mathgl_prepare_data.h"

int sample(mglGraph *gr)
{
  mglData x,y,y1,y2;  mgls_prepare1d(&y,&y1,&y2,&x);
  gr->SubPlot(1,1,0,""); gr->Title("Traj plot");
  gr->Box();  gr->Plot(x,y);  gr->Traj(x,y,y1,y2);
  return 0;
}

//-----------------------------------------------------
int main ( int argc,char **argv ) {
    mglQT gr ( sample,"MathGL examples" );
    return gr.Run();
}
