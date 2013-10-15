#ifndef __LEAPINTERACTION_H__
#define __LEAPINTERACTION_H__

#include "Leap.h"
#include "cinder/Vector.h"
#include "LeapListener.h"
#include "UserInterface.h"
#include "Sculpt.h"
#include "Utilities.h"
#include <cinder/app/App.h>

class LeapInteraction
{

public:

	LeapInteraction(Sculpt* _Sculptor, UserInterface* _Ui);
	void processInteraction(LeapListener& _Listener, float _Aspect, const Matrix44f& _Model, const Matrix44f& _Projection, const Vec2i& _Viewport, bool _Supress);

	float getDPhi() const { return _dphi; }
	float getDTheta() const { return _dtheta; }
	float getDZoom() const { return _dzoom; }
	void setBrushRadius(float _Radius) { _desired_brush_radius = _Radius; }
	void setBrushStrength(float _Strength) { _desired_brush_strength = _Strength; }

private:

	void interact();

	Leap::Frame _cur_frame;
	Leap::Frame _last_frame;

	Sculpt* _sculpt;
	UserInterface* _ui;
	std::vector<Vec4f> _tips;
	Matrix44f _model_view_inv;
	Matrix44f _model_view;
	Matrix44f _projection;
	Vec2i _window_size;
	float _desired_brush_radius;
	float _desired_brush_strength;
	float _dphi;
	float _dtheta;
	float _dzoom;

};

#endif
