#include "StdAfx.h"
#include "LeapInteraction.h"
#include "cinder/app/AppBasic.h"
#include "Utilities.h"
#include <vector>

using namespace ci;

LeapInteraction::LeapInteraction(Sculpt* _Sculpt, UserInterface* _Ui)
	: _sculpt(_Sculpt)
	, _ui(_Ui)
	, _desired_brush_radius(0.4f)
	, _dphi(0.0f)
	, _dtheta(0.0f)
	, _dzoom(0.0f)
  , _is_pinched(false)
{ }

void LeapInteraction::processInteraction(LeapListener& _Listener, float _Aspect, const Matrix44f& _Model, const Matrix44f& _Projection, const Vec2i& _Viewport, bool _Supress)
{
	_sculpt->clearBrushes();
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
	static const Vec3f LEAP_OFFSET(0, 200, 150);
	Leap::PointableList pointables = _cur_frame.pointables();
	const float ui_mult = 1.0f - _ui->maxActivation();
	const int num_pointables = pointables.count();
	for (int i=0; i<num_pointables; i++)
	{
		// add brushes
		const float strengthMult = Utilities::SmootherStep(math<float>::clamp(pointables[i].timeVisible()/AGE_WARMUP_TIME));
		Leap::Vector tip_pos = pointables[i].tipPosition();
		Leap::Vector tip_dir = pointables[i].direction();
    Leap::Vector tip_vel = pointables[i].tipVelocity();
		Vec3f pos = Vec3f(tip_pos.x, tip_pos.y, tip_pos.z) - LEAP_OFFSET;
		Vec3f dir = Vec3f(tip_dir.x, tip_dir.y, tip_dir.z);
    Vec3f vel = Vec3f(tip_vel.x, tip_vel.y, tip_vel.z);
		Vector3 brushPos(_model_view_inv.transformPoint(pos).ptr());
		Vector3 brushDir((-_model_view_inv.transformVec(dir)).ptr());
    Vector3 brushVel(_model_view_inv.transformVec(vel).ptr());
		_sculpt->addBrush(brushPos, brushDir, brushVel, _desired_brush_radius, strengthMult*ui_mult*_desired_brush_strength);

		Vec3f transPos = _projection.transformPoint(pos);
		Vec3f radPos = _projection.transformPoint(pos+Vec3f(_desired_brush_radius, 0, 0));

		// move camera based on finger distance from the center of the screen
		Vec2f diffXY(transPos.x, transPos.y);
		float lengthXY = diffXY.lengthSquared();
		float diffZ = (pos.z + LEAP_OFFSET.z);
		diffXY = diffXY.normalized();
		float lengthZ = diffZ*diffZ;
		static const float XY_NEUTRAL_RADIUS_SQ = 1.0f;
		static const float Z_NEUTRAL_RADIUS_SQ = 100.0f * 100.0f;
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
			cur_dzoom += 0.01f*ui_mult*mult*diffZ*ZOOM_SPEED;
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
		Vec4f tip(transPos.x, transPos.y, transPos.z, strengthMult);
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

  //// Handle pinching
  //const int numHands = _cur_frame.hands().count();
  //for (int ih = 0; ih < numHands; ih++) {
  //  Leap::Hand& hand = _cur_frame.hands()[ih];
  //  LM_LOG << "Manipulation strength " << float(hand.manipulationStrength()) << std::endl;
  //}

  static const float PINCH_START_THRESHOLD = 0.8f;
  static const float PINCH_END_THRESHOLD = 0.7f;
  // Handle pinching
  if (_is_pinched) {
    Leap::Hand& hand = _cur_frame.hand(_pinching_hand_id);
    // Handle pinch end
    if (!hand.isValid() || hand.manipulationStrength() < PINCH_END_THRESHOLD) {
      _is_pinched = false;
    } else {
      // Handle pinch drag
      _pinch_last_recorded = ToVec3f(hand.manipulationPoint().toVector3<Vector3>());

      // Check for pinning a movement direction
      if (!_pin_z && !_pin_xy) {
        Vec3f diff = _pinch_last_recorded - _pinch_origin;
        if (20.0f < diff.length()) {
          // Check angle and decide on direction
          if (std::sqrt(diff[0]*diff[0]+diff[1]*diff[1]) < std::fabs(diff[2]) ) {
            // use z
            _pin_xy = true;
          } else {
            // use xy
            _pin_z = true;
          }
        }
      }
    }
  } else {
    // Handle pinch start
    const int numHands = _cur_frame.hands().count();
    for (int ih = 0; ih < numHands; ih++) {
      Leap::Hand& hand = _cur_frame.hands()[ih];
      if (PINCH_START_THRESHOLD < hand.manipulationStrength()) {
        _is_pinched = true;
        _pin_z = false;
        _pin_xy = false;
        _pinching_hand_id = hand.id();
        _pinch_origin = ToVec3f(hand.manipulationPoint().toVector3<Vector3>());
        _pinch_last_read = _pinch_origin;
        _pinch_last_recorded = _pinch_origin;
      }
      //LM_LOG << "Manipulation strength " << float(hand.manipulationStrength()) << std::endl;
    }
  }
}

Vec3f LeapInteraction::getPinchDeltaFromLastCall() {
  if (!_is_pinched) {
    return Vec3f(0.0f, 0.0f, 0.0f);
  } else {
    Vec3f result = _pinch_last_recorded - _pinch_last_read;
    _pinch_last_read = _pinch_last_recorded;
    if (_pin_xy) {
      result[0] = 0.0f;
      result[1] = 0.0f;
    } 
    if (_pin_z) {
      result[2] = 0.0f;
    }
    return result;
  }
}

