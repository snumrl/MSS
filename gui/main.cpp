#include "Window.h"

int main(int argc,char** argv)
{
	glutInit(&argc, argv);
	MSS::Window* window = new MSS::Window();
	
	window->initWindow(1920,1080,"gui");
	glutMainLoop();
}
