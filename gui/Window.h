#ifndef __MSS_WINDOW_H__
#define __MSS_WINDOW_H__
#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"

namespace MSS
{
class Environment;
class Window : public dart::gui::Win3D
{
public:
	Window();

	void draw() override;
	void keyboard(unsigned char _key, int _x, int _y) override;
private:
	void DrawEntity(const dart::dynamics::Entity* entity) const;
	void DrawBodyNode(const dart::dynamics::BodyNode* bn) const;
	void DrawSkeleton(const dart::dynamics::SkeletonPtr& skel) const;
	void DrawShapeFrame(const dart::dynamics::ShapeFrame* shapeFrame) const;
	void DrawShape(const dart::dynamics::Shape* shape,const Eigen::Vector4d& color) const;

	void Step();
	void SetFocusing();

	Environment* mEnv;
	bool mFocus;
};
};


#endif