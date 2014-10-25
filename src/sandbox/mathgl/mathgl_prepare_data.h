
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
void mgls_prepare2d(HMDT a, HMDT b=0, HMDT v=0)
{
  register long i,j,n=50,m=40;
  if(a) mgl_data_create(a,n,m,1);
  if(b) mgl_data_create(b,n,m,1);
  if(v) { mgl_data_create(v,9,1,1); mgl_data_fill(v,-1,1,'x');  }
  mreal x,y;
  for(i=0;i<n;i++)  for(j=0;j<m;j++)
  {
    x = i/(n-1.); y = j/(m-1.);
    if(a) mgl_data_set_value(a, 0.6*sin(2*M_PI*x)*sin(3*M_PI*y)+0.4*cos(3*M_PI*x*y), i,j,0);
    if(b) mgl_data_set_value(b, 0.6*cos(2*M_PI*x)*cos(3*M_PI*y)+0.4*cos(3*M_PI*x*y), i,j,0);
  }
}

void mgls_prepare2v(HMDT a, HMDT b)
{
  register long i,j,n=20,m=30;
  if(a) mgl_data_create(a,n,m,1);
  if(b) mgl_data_create(b,n,m,1);
  mreal x,y;
  for(i=0;i<n;i++)  for(j=0;j<m;j++)
  {
    x=i/(n-1.); y=j/(m-1.);
    if(a) mgl_data_set_value(a, 0.6*sin(2*M_PI*x)*sin(3*M_PI*y)+0.4*cos(3*M_PI*x*y), i,j,0);
    if(b) mgl_data_set_value(b, 0.6*cos(2*M_PI*x)*cos(3*M_PI*y)+0.4*cos(3*M_PI*x*y), i,j,0);
  }
}
void mgls_prepare3v(HMDT ex, HMDT ey, HMDT ez)
{
  register long i,j,k,n=10;
  if(!ex || !ey || !ez) return;
  mgl_data_create(ex,n,n,n);
  mgl_data_create(ey,n,n,n);
  mgl_data_create(ez,n,n,n);
  mreal x,y,z, r1,r2;
  for(i=0;i<n;i++)  for(j=0;j<n;j++)  for(k=0;k<n;k++)
  {
    x=2*i/(n-1.)-1; y=2*j/(n-1.)-1; z=2*k/(n-1.)-1;
    r1 = pow(x*x+y*y+(z-0.3)*(z-0.3)+0.03,1.5);
    r2 = pow(x*x+y*y+(z+0.3)*(z+0.3)+0.03,1.5);
    mgl_data_set_value(ex, 0.2*x/r1 - 0.2*x/r2, i,j,k);
    mgl_data_set_value(ey, 0.2*y/r1 - 0.2*y/r2, i,j,k);
    mgl_data_set_value(ez, 0.2*(z-0.3)/r1 - 0.2*(z+0.3)/r2, i,j,k);
  }
}
