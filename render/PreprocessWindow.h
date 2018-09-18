#ifndef __MSS_PREPROCESS_WINDOW_H__
#define __MSS_PREPROCESS_WINDOW_H__
#include "Camera.h"
#include "GLUTWindow.h"
#include "GLfunctions.h"
#include "MSSInterface.h"
#include "Environment.h"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <string>

namespace p = boost::python;
namespace np = boost::python::numpy;

class PreprocessWindow : public GUI::GLUTWindow
{
public:
	PreprocessWindow();
	
protected:

	/// Draw all the skeletons in mWorld. Lights and Camera are operated here.
	void Display() override;

	/// The user interactions with keyboard.
	void Keyboard(unsigned char key,int x,int y) override;

	/// Stores the data for SimWindow::Motion.
	void Mouse(int button, int state, int x, int y) override;

	/// The user interactions with mouse. Camera view is set here.
	void Motion(int x, int y) override;

	/// Reaction to window resizing.
	void Reshape(int w, int h) override;

	/// 
	void Timer(int value) override;

	/// Screenshot. The png file will be stored as ./frames/Capture/[number].png
	void Screenshot();
	void GetFromNN();

	Eigen::VectorXd s,qdd_des,activation,b,q,q_d,v,v_d;
	Eigen::VectorXd q_next,v_next;
	Eigen::MatrixXd A;
	MSS::Environment* mWorld;

	bool mIsRotate;
	bool mIsCapture;
	bool mIsNNLoaded;
	int mNumTuple;
	//python object
	p::object mm,mns,sys_module,reg_module; //main module,main namespace,sys module,nn_module

};

#endif