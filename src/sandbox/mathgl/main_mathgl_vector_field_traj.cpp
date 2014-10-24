
#include <mgl2/qt.h>
void mgls_prepare1d(HMDT y, HMDT y1=0, HMDT y2=0, HMDT x1=0, HMDT x2=0)
{
  register long i,n=50;
  if(y)   mgl_data_create(y,n,3,1);
  if(x1)  mgl_data_create(x1,n,1,1);
  if(x2)  mgl_data_create(x2,n,1,1);
  if(y1)  mgl_data_create(y1,n,1,1);
  if(y2)  mgl_data_create(y2,n,1,1);
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
    if(y1)  mgl_data_set_value(y1, 0.5+0.3*cos(2*M_PI*xx), i,0,0);
    if(y2)  mgl_data_set_value(y2, 0.3*sin(2*M_PI*xx), i,0,0);
    if(x1)  mgl_data_set_value(x1, xx*2-1, i,0,0);
    if(x2)  mgl_data_set_value(x2, 0.05+0.03*cos(2*M_PI*xx), i,0,0);
  }
}
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
