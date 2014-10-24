#include <mgl2/qt.h>
class Foo : public mglDraw
{
public:
  int Draw(mglGraph *gr);
};
//-----------------------------------------------------
int Foo::Draw(mglGraph *gr)
{
  gr->Rotate(60,40);
  gr->Box();
  return 0;
}
//-----------------------------------------------------
int main(int argc,char **argv)
{
  Foo foo;
  mglQT gr(&foo,"MathGL examples");
  return gr.Run();
}
