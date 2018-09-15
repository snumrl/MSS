#include "SimWindow.h"
#include <vector>
#include <string>
#include <GL/glut.h>

namespace p = boost::python;
namespace np = boost::python::numpy;

int main(int argc,char** argv)
{
	Py_Initialize();
	np::initialize();
	std::cout<<"` : mIsRotate= !mIsRotate;"<<std::endl;
	std::cout<<"C : mIsCapture = true; "<<std::endl;
	std::cout<<"] : mWorld->Step();"<<std::endl;
	std::cout<<"R : mWorld->Reset(false);"<<std::endl;
	std::cout<<"r : mWorld->Reset(true);"<<std::endl;
	std::cout<<"f : mIsFocusing=!mIsFocusing;"<<std::endl;
	std::cout<<"1 : mFocusBodyNum = 0;"<<std::endl;
	std::cout<<"2 : mFocusBodyNum = foot->GetSkeleton()->getBodyNode(TalusR)->getIndexInSkeleton();"<<std::endl;
	std::cout<<"3 : mFocusBodyNum = foot->G1etSkeleton()->getBodyNode(TalusL)->getIndexInSkeleton();"<<std::endl;
	std::cout<<"- : mLB[mLBCount]+=0.01;"<<std::endl;
	std::cout<<"+ : mLB[mLBCount]-=0.01;"<<std::endl;
	std::cout<<"q : std::cout<<mWorld->GetState().transpose()<<std::endl;"<<std::endl;
	std::cout<<"w : std::cout<<mWorld->GetReward()<<std::endl;"<<std::endl;
	std::cout<<"b : mLBCount++;"<<std::endl;
	std::cout<<"B : mLBCount--;"<<std::endl;
	std::cout<<"  : mIsAuto = !mIsAuto;"<<std::endl;
	std::cout<<"7 : exit(0);"<<std::endl;
	SimWindow* simwindow;
	if(argc==1)
		simwindow = new SimWindow();
	else
		simwindow = new SimWindow(argv[1],argv[2]);

	glutInit(&argc, argv);
	simwindow->InitWindow(1920,1080,"Render");
	glutMainLoop();
}
