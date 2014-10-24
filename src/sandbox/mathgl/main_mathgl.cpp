
#include <mgl2/qt.h>

void mgls_prepare1d(HMDT y, HMDT y1=0, HMDT y2=0, HMDT x1=0, HMDT x2=0)
{
  register long i,n=50;
  if(y)   mgl_data_create(y,n,3,1);
  mreal xx;
  for(i=0;i<n;i++)
  {
    xx = i/(n-1.);
    if(y)
    {
      mgl_data_set_value(y, 0.7*sin(2*M_PI*xx) + 0.5*cos(3*M_PI*xx) + 0.2*sin(M_PI*xx), i,0,0);
      mgl_data_set_value(y, sin(2*M_PI*xx), i,1,0);
      mgl_data_set_value(y, cos(2*M_PI*xx), i,2,0);
    }
  }
}

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
