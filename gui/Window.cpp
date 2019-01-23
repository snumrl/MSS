#include "Window.h"

using namespace MSS;
using namespace dart;
using namespace dart::gui;


Window::
Window()
{
	mBackground[0] = 1.0;
	mBackground[1] = 1.0;
	mBackground[2] = 1.0;
	mBackground[3] = 1.0;
}
void
Window::
draw()
{
	glPushMatrix();
	glutSolidCube(1.0);
	glPopMatrix();
}
