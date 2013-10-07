#include "LeapInteraction.h"
#include "cinder/app/AppBasic.h"
#include "Utilities.h"
#include <vector>

using namespace ci;

LeapInteraction::LeapInteraction(Sculptor* _Sculptor, UserInterface* _Ui)
	: _sculptor(_Sculptor)
	, _ui(_Ui)
	, _desired_brush_radius(0.4f)
	, _dphi(0.0f)
	, _dtheta(0.0f)
	, _dzoom(0.0f)
{ }

void LeapInteraction::processInteraction(LeapListener& _Listener, float _Aspect, const Matrix44f& _Model, const Matrix44f& _Projection, const Vec2i& _Viewport, bool _Supress)
{
	_sculptor->clearBrushes();
	_tips.clear();
	_model_view_inv = _Model.inverted();
	_model_view = _Model;
	_projection = _Projection;
	_window_size = _Viewport;
	if (_Supress)
	{
		_cur_frame = Leap::Frame::invalid();
	}
	else if (_Listener.isConnected() && _Listener.waitForFrame(_cur_frame, 30))
	{
		interact();
		_last_frame = _cur_frame;
	}
	_ui->update(_tips);
}

void LeapInteraction::interact()
{
	float cur_dtheta = 0;
	float cur_dphi = 0;
	float cur_dzoom = 0;
	static const float ORBIT_SPEED = 0.02f;
	static const float ZOOM_SPEED = 0.5f;
	static const float AGE_WARMUP_TIME = 0.75;

	// create brushes
	static const float LEAP_SCALE = 0.01f;
	static const Vec3f LEAP_OFFSET = LEAP_SCALE*Vec3f(0, 200, 150);
	Leap::PointableList pointables = _cur_frame.pointables();
	const float ui_mult = 1.0f - _ui->maxActivation();
	const int num_pointables = pointables.count();
	for (int i=0; i<num_pointables; i++)
	{
		// add brushes
		const float weight = Utilities::SmootherStep(math<float>::clamp(pointables[i].timeVisible()/AGE_WARMUP_TIME));
		Leap::Vector tip_pos = pointables[i].tipPosition();
		Leap::Vector tip_dir = pointables[i].direction();
		Vec3f pos = LEAP_SCALE*Vec3f(tip_pos.x, tip_pos.y, tip_pos.z) - LEAP_OFFSET;
		Vec3f dir = Vec3f(tip_dir.x, tip_dir.y, tip_dir.z);
		_sculptor->addBrush(_model_view_inv.transformPoint(pos), -_model_view_inv.transformVec(dir), _desired_brush_radius, ui_mult*_desired_brush_strength, weight);

		Vec3f transPos = _projection.transformPoint(pos);
		Vec3f radPos = _projection.transformPoint(pos+Vec3f(_desired_brush_radius, 0, 0));

		// move camera based on finger distance from the center of the screen
		Vec2f diffXY(transPos.x, transPos.y);
		float lengthXY = diffXY.lengthSquared();
		float diffZ = (pos.z + LEAP_OFFSET.z);
		diffXY = diffXY.normalized();
		float lengthZ = diffZ*diffZ;
		static const float XY_NEUTRAL_RADIUS_SQ = 1.0f;
		static const float Z_NEUTRAL_RADIUS_SQ = (100.0f*LEAP_SCALE)*(100.0f*LEAP_SCALE);
		static const float BORDER_MULT = 6.0f;
		if (lengthXY > XY_NEUTRAL_RADIUS_SQ && lengthXY < BORDER_MULT*XY_NEUTRAL_RADIUS_SQ)
		{
			float mult = math<float>::clamp(lengthXY - XY_NEUTRAL_RADIUS_SQ);
			cur_dtheta += -ui_mult*mult*diffXY.x*ORBIT_SPEED;
			cur_dphi += ui_mult*mult*diffXY.y*ORBIT_SPEED;
		}
		if (lengthZ > Z_NEUTRAL_RADIUS_SQ && lengthZ < BORDER_MULT*Z_NEUTRAL_RADIUS_SQ)
		{
			float mult = math<float>::clamp(lengthZ - Z_NEUTRAL_RADIUS_SQ);
			cur_dzoom += ui_mult*mult*diffZ*ZOOM_SPEED;
		}

		// compute screen-space coordinate of this finger
		transPos.x = (transPos.x + 1)/2;
		transPos.y = (transPos.y + 1)/2;
		transPos.z = 1.0f;

		// compute a point on the surface of the sphere to use as the screen-space radius
		radPos.x = (radPos.x + 1)/2;
		radPos.y = (radPos.y + 1)/2;
		radPos.z = 1.0f;
		float rad = transPos.distance(radPos);
		transPos.z = rad;
		Vec4f tip(transPos.x, transPos.y, transPos.z, weight);
		_tips.push_back(tip);
	}

	if (num_pointables > 0)
	{
		cur_dtheta /= num_pointables;
		cur_dphi /= num_pointables;
		cur_dzoom /= num_pointables;
	}

	static const float SMOOTH_STRENGTH = 0.9f;
	_dtheta = SMOOTH_STRENGTH*_dtheta + (1.0f-SMOOTH_STRENGTH)*cur_dtheta;
	_dphi = SMOOTH_STRENGTH*_dphi + (1.0f-SMOOTH_STRENGTH)*cur_dphi;
	_dzoom = SMOOTH_STRENGTH*_dzoom + (1.0f-SMOOTH_STRENGTH)*cur_dzoom;
}
