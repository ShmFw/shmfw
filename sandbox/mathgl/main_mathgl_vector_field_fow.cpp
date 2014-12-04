
#include <mgl2/qt.h>
#include "mathgl_prepare_data.h"

int sample(mglGraph *gr)
{
  mglData a,b;  mgls_prepare2v(&a,&b);
  gr->SubPlot(2,2,0,""); gr->Title("Flow plot (default)");
  gr->Box();  gr->Flow(a,b);

  gr->SubPlot(2,2,1,"");  gr->Title("'v' style");
  gr->Box();  gr->Flow(a,b,"v");

  gr->SubPlot(2,2,2,"");  gr->Title("'\\#' style");
  gr->Box();  gr->Flow(a,b,"#");

  mglData ex,ey,ez; mgls_prepare3v(&ex,&ey,&ez);
  gr->SubPlot(2,2,3); gr->Title("3d variant");  gr->Rotate(50,60);
  gr->Box();  gr->Flow(ex,ey,ez);
  return 0;
}

//-----------------------------------------------------
int main ( int argc,char **argv ) {
    mglQT gr ( sample,"MathGL examples" );
    return gr.Run();
}
