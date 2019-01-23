#ifndef __MSS_WINDOW_H__
#define __MSS_WINDOW_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"

namespace MSS
{
class Window : public dart::gui::Win3D
{
public:
	Window();
	void draw();
};
};


#endif