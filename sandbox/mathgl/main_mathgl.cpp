
#include <mgl2/qt.h>
#include "mathgl_prepare_data.h"

int sample(mglGraph *gr)
{
  mglData y; mgls_prepare1d(&y); gr->SetOrigin(0,0,0);
  gr->SubPlot(2,2,0,"");  gr->Title("Tens plot (default)");
  gr->Box();  gr->Tens(y.SubData(-1,0), y.SubData(-1,1));

  gr->SubPlot(2,2,2,"");  gr->Title("' ' style");
  gr->Box();  gr->Tens(y.SubData(-1,0), y.SubData(-1,1),"o ");

  gr->SubPlot(2,2,1); gr->Title("3d variant");  gr->Rotate(50,60);  gr->Box();
  mglData yc(30), xc(30), z(30);  z.Modify("2*x-1");
  yc.Modify("sin(pi*(2*x-1))"); xc.Modify("cos(pi*2*x-pi)");
  gr->Tens(xc,yc,z,z,"s");
  return 0;
}
//-----------------------------------------------------
int main ( int argc,char **argv ) {
    mglQT gr ( sample,"MathGL examples" );
    return gr.Run();
}
