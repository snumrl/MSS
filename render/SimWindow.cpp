#include "SimWindow.h"
#include "dart/external/lodepng/lodepng.h"
#include "SkeletonBuilder.h"
#include "Functions.h"
#include <algorithm>
#include <fstream>
#include <boost/filesystem.hpp>
#include <GL/glut.h>
using namespace GUI;
using namespace dart::simulation;
using namespace dart::dynamics;
SimWindow::
SimWindow()
	:GLUTWindow(),mIsRotate(false),mIsAuto(false),mIsCapture(false),mFocusBodyNum(0),mIsFocusing(false),mIsNNLoaded(false),mIsMuscleNNLoaded(false),mActionNum(0),mRandomAction(false)
{
	mWorld = new MSS::Environment(30,600);
	mAction =Eigen::VectorXd::Zero(mWorld->GetNumAction());
	mDisplayTimeout = 33;
}
SimWindow::
SimWindow(const std::string& nn_path)
	:SimWindow()
{
	mIsNNLoaded = true;

	mm = p::import("__main__");
	mns = mm.attr("__dict__");
	sys_module = p::import("sys");
	p::str module_dir = (std::string(MSS_ROOT_DIR)+"/pymss").c_str();
	sys_module.attr("path").attr("insert")(1, module_dir);
	p::exec("import torch",mns);
	p::exec("import torch.nn as nn",mns);
	p::exec("import torch.optim as optim",mns);
	p::exec("import torch.nn.functional as F",mns);
	p::exec("import torchvision.transforms as T",mns);
	p::exec("import numpy as np",mns);
	p::exec("from Model import *",mns);

	boost::python::str str = ("num_state = "+std::to_string(mWorld->GetNumState())).c_str();
	p::exec(str,mns);
	str = ("num_action = "+std::to_string(mWorld->GetNumAction())).c_str();
	p::exec(str,mns);

	nn_module = p::eval("SimulationNN(num_state,num_action)",mns);

	p::object load = nn_module.attr("load");
	load(nn_path);
}

SimWindow::
SimWindow(const std::string& nn_path,const std::string& muscle_nn_path)
	:SimWindow(nn_path)
{
	mIsMuscleNNLoaded = true;

	boost::python::str str = ("num_state = "+std::to_string(mWorld->GetNumState())).c_str();
	p::exec(str,mns);
	str = ("num_dofs = "+std::to_string(mWorld->GetCharacter()->GetSkeleton()->getNumDofs())).c_str();
	p::exec(str,mns);
	str = ("num_muscles = "+std::to_string(mWorld->GetCharacter()->GetMuscles().size())).c_str();
	p::exec(str,mns);

	muscle_nn_module = p::eval("MuscleNN(num_state,num_dofs,num_muscles).cuda()",mns);

	p::object load = muscle_nn_module.attr("load");
	load(muscle_nn_path);
}

void
SimWindow::
Display() 
{
	glClearColor(1.0, 1.0, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	initLights();

	bool exist_humanoid = false;
	auto character = mWorld->GetCharacter();
	auto ground = mWorld->GetGround();
	if(mIsFocusing)
	{
		Eigen::Isometry3d T = character->GetSkeleton()->getBodyNode(mFocusBodyNum)->getTransform();
		mCamera->SetLookAt(T.translation());
	}
	mCamera->Apply();
	glDisable(GL_LIGHTING);
	glColor3f(0.8,0.8,0.8);
	
	float y = 
		ground->getBodyNode(0)->getTransform().translation()[1] +
		dynamic_cast<const BoxShape*>(ground->getBodyNode(0)->getShapeNodesWith<dart::dynamics::VisualAspect>()[0]->getShape().get())->getSize()[1]*0.5;
	glColor3f(0,0,0);
	glLineWidth(1.0);
	Eigen::Vector3d com = character->GetSkeleton()->getCOM();
	double ox,oz;
	ox = std::fmod(com[0],0.5);
	oz = std::fmod(com[2],0.5);
	ox = com[0]-ox;
	oz = com[2]-oz;
	
	glPushMatrix();
	glTranslatef(ox,0,oz);
	glBegin(GL_LINES);
	for(float x =-20.0;x<=20.0001;x+=0.5)
	{
		glVertex3f(x,y,-20.0);
		glVertex3f(x,y,20.0);
	}
	for(float x =-20.0;x<=20.0001;x+=0.5)
	{
		glVertex3f(-20.0,y,x);
		glVertex3f(20.0,y,x);
	}
	glEnd();
	glPopMatrix();
	{
		GUI::DrawSkeleton(character->GetSkeleton());
		GUI::DrawMuscles(character->GetMuscles());
		auto cps = character->GetContactPoints();

		for(auto cp : cps)
		{
			glPushMatrix();
			Eigen::Vector3d pos = cp->GetPosition();
			if(cp->IsColliding())
				glColor3f(0.8,0.2,0.2);
			else
				glColor3f(0.2,0.2,0.8);

			glTranslatef(pos[0],pos[1],pos[2]);
			GUI::DrawSphere(0.004);
			glPopMatrix();

		}
	}
	{
		Eigen::VectorXd p_save = character->GetSkeleton()->getPositions();
		Eigen::VectorXd action_motion = mWorld->GetAction().head(character->GetMotionActions().size());
		// Eigen::VectorXd action_IK = mWorld->GetAction().segment(character->GetMotionActions().size(),character->GetIKAction()->GetDof());
		auto target = character->GetTargetPositionsAndVelocitiesFromBVH(action_motion);
		// target.first = character->GetIKTargetPositions(target.first,action_IK);
		target.first[3] += 3.0;
		character->GetSkeleton()->setPositions(target.first);
		character->GetSkeleton()->computeForwardKinematics(true,false,false);
		GUI::DrawSkeleton(character->GetSkeleton(),Eigen::Vector3d(0.2,0.8,0.2));
		auto cps = character->GetContactPoints();

		for(auto cp : cps)
		{
			glPushMatrix();
			Eigen::Vector3d pos = cp->GetPosition();
			if(cp->IsColliding())
				glColor3f(0.8,0.2,0.2);
			else
				glColor3f(0.2,0.2,0.8);

			glTranslatef(pos[0],pos[1],pos[2]);
			GUI::DrawSphere(0.004);
			glPopMatrix();

		}
		character->GetSkeleton()->setPositions(p_save);
		character->GetSkeleton()->computeForwardKinematics(true,false,false);
	}
	{
		Eigen::VectorXd p_save = character->GetSkeleton()->getPositions();
		Eigen::VectorXd p = character->GetTargetPositions();
		p[3] +=1.5;
		character->GetSkeleton()->setPositions(p);
		character->GetSkeleton()->computeForwardKinematics(true,false,false);
		GUI::DrawSkeleton(character->GetSkeleton(),Eigen::Vector3d(0.8,0.2,0.2));

		auto cps = character->GetContactPoints();

		for(auto cp : cps)
		{
			glPushMatrix();
			Eigen::Vector3d pos = cp->GetPosition();
			if(cp->IsColliding())
				glColor3f(0.8,0.2,0.2);
			else
				glColor3f(0.2,0.2,0.8);

			glTranslatef(pos[0],pos[1],pos[2]);
			GUI::DrawSphere(0.004);
			glPopMatrix();

		}
		character->GetSkeleton()->setPositions(p_save);
		character->GetSkeleton()->computeForwardKinematics(true,false,false);
	}

	glutSwapBuffers();
	if(mIsCapture)
		Screenshot();
	glutPostRedisplay();
}

void
SimWindow::
Keyboard(unsigned char key,int x,int y) 
{
	auto character = mWorld->GetCharacter();
	switch(key)
	{
		case '`': mIsRotate= !mIsRotate;break;
		case 'C': mIsCapture = true; break;
	
		case ']': Step();break;
		case 'R': mWorld->Reset(false);break;
		case 'r': mWorld->Reset(true);break;
		case 'm': mAction.setZero();
		case 't': mRandomAction=!mRandomAction;break;
		case 'f': mIsFocusing=!mIsFocusing;break;
		case '1': mFocusBodyNum = 0;break;
		case '2': mFocusBodyNum = character->GetSkeleton()->getBodyNode("TalusR")->getIndexInSkeleton();break;
		case '3': mFocusBodyNum = character->GetSkeleton()->getBodyNode("TalusL")->getIndexInSkeleton();break;
		case 'q': std::cout<<mWorld->GetState().transpose()<<std::endl;break;
		case ' ': mIsAuto = !mIsAuto;break;
		case '+': mAction[mActionNum]+=0.3;break;
		case '-': mAction[mActionNum]-=0.3;break;
		case 'b': mActionNum++;mActionNum %= mAction.size();break;
		case 27 : exit(0);break;
		default : break;
	}
	std::cout<<mActionNum<<std::endl;
	
	
	
	glutPostRedisplay();
}
void
SimWindow::
Mouse(int button, int state, int x, int y) 
{
	if (state == GLUT_DOWN)
	{
		mIsDrag = true;
		mMouseType = button;
		mPrevX = x;
		mPrevY = y;
	}
	else
	{
		mIsDrag = false;
		mMouseType = 0;
	}

	glutPostRedisplay();
}
void
SimWindow::
Motion(int x, int y) 
{
	if (!mIsDrag)
		return;

	int mod = glutGetModifiers();
	if (mMouseType == GLUT_LEFT_BUTTON)
	{
		if(!mIsRotate)
		mCamera->Translate(x,y,mPrevX,mPrevY);
		else
		mCamera->Rotate(x,y,mPrevX,mPrevY);
	}
	else if (mMouseType == GLUT_RIGHT_BUTTON)
	{
		switch (mod)
		{
		case GLUT_ACTIVE_SHIFT:
			mCamera->Zoom(x,y,mPrevX,mPrevY); break;
		default:
			mCamera->Pan(x,y,mPrevX,mPrevY); break;		
		}

	}
	mPrevX = x;
	mPrevY = y;
	glutPostRedisplay();
}
void
SimWindow::
Reshape(int w, int h) 
{
	glViewport(0, 0, w, h);
	mCamera->Apply();
}
void
SimWindow::
Step()
{
	GetActionFromNN();
	mWorld->SetAction(mAction);
	int sim_per_control = mWorld->GetSimulationHz()/mWorld->GetControlHz();
	for(int i =0;i<sim_per_control;i++){
		// Eigen::VectorXd activation = Eigen::VectorXd::Zero(mWorld->GetCharacter()->GetMuscles().size());
		Eigen::VectorXd activation = (mIsMuscleNNLoaded?GetActivationFromNN():mWorld->ComputeActivationQP());
		mWorld->Step(activation);		
		
	}
}
void
SimWindow::
Timer(int value) 
{
	if( mIsAuto ) Step();
	
	glutTimerFunc(mDisplayTimeout, TimerEvent,1);
	glutPostRedisplay();
}


void SimWindow::
Screenshot() {
  static int count = 0;
  const char directory[8] = "frames";
  const char fileBase[8] = "Capture";
  char fileName[32];

  boost::filesystem::create_directories(directory);
  std::snprintf(fileName, sizeof(fileName), "%s%s%s%.4d.png",
                directory, "/", fileBase, count++);
  int tw = glutGet(GLUT_WINDOW_WIDTH);
  int th = glutGet(GLUT_WINDOW_HEIGHT);

  glReadPixels(0, 0,  tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);

  // reverse temp2 temp1
  for (int row = 0; row < th; row++) {
    memcpy(&mScreenshotTemp2[row * tw * 4],
           &mScreenshotTemp[(th - row - 1) * tw * 4], tw * 4);
  }

  unsigned result = lodepng::encode(fileName, mScreenshotTemp2, tw, th);

  // if there's an error, display it
  if (result) {
    std::cout << "lodepng error " << result << ": "
              << lodepng_error_text(result) << std::endl;
    return ;
  } else {
    std::cout << "wrote screenshot " << fileName << "\n";
    return ;
  }
}
np::ndarray toNumPyArray(const Eigen::VectorXd& vec)
{
	int n = vec.rows();
	p::tuple shape = p::make_tuple(n);
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray array = np::empty(shape,dtype);

	float* dest = reinterpret_cast<float*>(array.get_data());
	for(int i =0;i<n;i++)
	{
		dest[i] = vec[i];
	}

	return array;
}
void
SimWindow::
GetActionFromNN()
{
	if(!mIsNNLoaded)
		return;
	p::object get_action;
	if(mRandomAction)
		get_action= nn_module.attr("get_random_action");
	else
		get_action= nn_module.attr("get_action");
	Eigen::VectorXd state = mWorld->GetState();
	p::tuple shape = p::make_tuple(state.rows());
	np::dtype dtype = np::dtype::get_builtin<float>();
	np::ndarray state_np = np::empty(shape,dtype);
	
	float* dest = reinterpret_cast<float*>(state_np.get_data());
	for(int i =0;i<state.rows();i++)
		dest[i] = state[i];
	
	p::object temp = get_action(state_np);
	np::ndarray action_np = np::from_object(temp);

	float* srcs = reinterpret_cast<float*>(action_np.get_data());
	for(int i=0;i<mAction.rows();i++)
		mAction[i] = srcs[i];
}
Eigen::VectorXd
SimWindow::
GetActivationFromNN()
{
	if(!mIsMuscleNNLoaded)
		return Eigen::VectorXd::Zero(0);
	p::object get_activation = muscle_nn_module.attr("get_activation");
	Eigen::VectorXd state = mWorld->GetState();
	Eigen::VectorXd qdd_des = mWorld->GetDesiredAcceleration();
	np::ndarray s_np = toNumPyArray(state);
	np::ndarray qdd_np = toNumPyArray(qdd_des);

	p::object temp = get_activation(s_np,qdd_np);
	np::ndarray activation_np = np::from_object(temp);

	Eigen::VectorXd activation(mWorld->GetCharacter()->GetMuscles().size());
	float* srcs = reinterpret_cast<float*>(activation_np.get_data());
	for(int i=0;i<activation.rows();i++)
		activation[i] = srcs[i];

	return activation;
}