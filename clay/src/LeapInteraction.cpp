#include "StdAfx.h"
#include "LeapInteraction.h"
#include "cinder/app/AppBasic.h"
#include "Utilities.h"
#include <vector>

using namespace ci;

#define USE_SKELETON_API 0

LeapInteraction::LeapInteraction(Sculpt* _Sculpt, UserInterface* _Ui)
  : _sculpt(_Sculpt)
  , _ui(_Ui)
  , _desired_brush_radius(0.4f)
  , _is_pinched(false)
  , _detailMode(false)
{
  _dphi.value = 0.0f;
  _dtheta.value = 0.0f;
  _dzoom.value = 0.0f;
}

bool LeapInteraction::processInteraction(LeapListener& _Listener, float _Aspect, const Matrix44f& _Model, const Matrix44f& _Projection, const Vec2i& _Viewport, bool _Supress)
{
  _model_view_inv = _Model.inverted();
  _model_view = _Model;
  _projection = _Projection;
  _window_size = _Viewport;
  if (_Supress)
  {
    _cur_frame = Leap::Frame::invalid();
  }
  else if (_Listener.isConnected() && _Listener.waitForFrame(_cur_frame, 33))
  {
    boost::unique_lock<boost::mutex> brushLock(_sculpt->getBrushMutex());
    _sculpt->clearBrushes();
    boost::unique_lock<boost::mutex> tipsLock(_tips_mutex);
    _tips.clear();
    interact();
    _last_frame = _cur_frame;
    return true;
  }
  return false;
}

void LeapInteraction::interact()
{
  float cur_dtheta = 0;
  float cur_dphi = 0;
  float cur_dzoom = 0;
  static const float ORBIT_SPEED = 0.006f;
  static const float ZOOM_SPEED = 50.0f;
  static const float AGE_WARMUP_TIME = 0.75;
  static const float TARGET_DELTA_TIME = 1.0f / 60.0f;
  static const float HAND_INFLUENCE_WARMUP = 0.333f; // time in seconds to reach full strength

  const double curTime = static_cast<double>(Utilities::TIME_STAMP_TICKS_TO_SECS * _cur_frame.timestamp());

  // create brushes
  static const Vec3f LEAP_OFFSET(0, 200, 100);
  Leap::HandList hands = _cur_frame.hands();
  const float ui_mult = 1.0f - _ui->maxActivation();
  const int num_hands = hands.count();
  const float deltaTime = static_cast<float>(Utilities::TIME_STAMP_TICKS_TO_SECS*(_cur_frame.timestamp() - _last_frame.timestamp()));
  const float dtMult = deltaTime / TARGET_DELTA_TIME;

  for (int i=0; i<num_hands; i++) {
    Leap::PointableList pointables = hands[i].pointables();
    Leap::Vector movement;
    const int num_pointables = pointables.count();
    //if (num_pointables > 2) {
    if (paddleTranslation(hands[i], _last_frame, movement)) {
      // camera interaction
      const float warmupMult = std::min(1.0f, hands[i].timeVisible()/HAND_INFLUENCE_WARMUP);
      movement *= warmupMult;
      cur_dtheta += ORBIT_SPEED * movement.x;
      cur_dphi += ORBIT_SPEED * -movement.y;
      cur_dzoom += ZOOM_SPEED * -movement.z;
    } else {
      for (int j=0; j<num_pointables; j++) {
        // add brushes
        const float strengthMult = Utilities::SmootherStep(math<float>::clamp(pointables[j].timeVisible()/AGE_WARMUP_TIME));
        Leap::Vector tip_pos = pointables[j].tipPosition();
        Leap::Vector tip_dir = pointables[j].direction();
        Leap::Vector tip_vel = pointables[j].tipVelocity();
        Vec3f pos = Vec3f(tip_pos.x, tip_pos.y, tip_pos.z) - LEAP_OFFSET;
        Vec3f dir = Vec3f(tip_dir.x, tip_dir.y, tip_dir.z);
        Vec3f vel = Vec3f(tip_vel.x, tip_vel.y, tip_vel.z);
        Vector3 brushPos(_model_view_inv.transformPoint(pos).ptr());
        Vector3 brushDir((-_model_view_inv.transformVec(dir)).ptr());
        Vector3 brushVel(_model_view_inv.transformVec(vel).ptr());
        float strength = strengthMult*ui_mult*_desired_brush_strength;
        strength = std::min(1.0f, strength * dtMult);

        Vec3f transPos = _projection.transformPoint(pos);
        Vec3f radPos = _projection.transformPoint(pos+Vec3f(_desired_brush_radius, 0, 0));

        // compute screen-space coordinate of this finger
        transPos.x = (transPos.x + 1)/2;
        transPos.y = (transPos.y + 1)/2;
        transPos.z = 1.0f;

        if (transPos.x >= 0.0f && transPos.x <= 1.0f && transPos.y >= 0.0f && transPos.y <= 1.0f) {
          _sculpt->addBrush(brushPos, brushDir, brushVel, _desired_brush_radius, strength);
        }

        // compute a point on the surface of the sphere to use as the screen-space radius
        radPos.x = (radPos.x + 1)/2;
        radPos.y = (radPos.y + 1)/2;
        radPos.z = 1.0f;
        float rad = transPos.distance(radPos);
        transPos.z = rad;
        Vec4f tip(transPos.x, transPos.y, transPos.z, strengthMult);
        _tips.push_back(tip);
      }
    }
  }

  cur_dtheta /= deltaTime;
  cur_dphi /= deltaTime;
  cur_dzoom /= deltaTime;

  static const float SMOOTH_STRENGTH = 0.9f;
  _dtheta.Update(cur_dtheta, curTime, SMOOTH_STRENGTH);
  _dphi.Update(cur_dphi, curTime, SMOOTH_STRENGTH);
  _dzoom.Update(cur_dzoom, curTime, SMOOTH_STRENGTH);

#if USE_SKELETON_API
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
#endif
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

