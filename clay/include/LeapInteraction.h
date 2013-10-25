#ifndef __LEAPINTERACTION_H__
#define __LEAPINTERACTION_H__

#include "Leap.h"
#include "cinder/Vector.h"
#include "LeapListener.h"
#include "UserInterface.h"
#include "Sculpt.h"
#include "Utilities.h"
#include <cinder/app/App.h>
#include <cinder/Thread.h>

class LeapInteraction
{

public:

  LeapInteraction(Sculpt* _Sculptor, UserInterface* _Ui);
  bool processInteraction(LeapListener& _Listener, float _Aspect, const Matrix44f& _Model, const Matrix44f& _Projection, const Vec2i& _Viewport, bool _Supress);

  float getDPhi() const { return _dphi; }
  float getDTheta() const { return _dtheta; }
  float getDZoom() const { return _dzoom; }
  Vec3f getPinchDeltaFromLastCall();
  bool isPinched() const { return _is_pinched; }
  void setBrushRadius(float _Radius) { _desired_brush_radius = _Radius; }
  void setBrushStrength(float _Strength) { _desired_brush_strength = _Strength; }
  double mostRecentTime() const { return Utilities::TIME_STAMP_TICKS_TO_SECS*_cur_frame.timestamp(); }
  std::vector<Vec4f> getTips() { boost::unique_lock<boost::mutex> tipsLock(_tips_mutex); return _tips; }

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
  boost::mutex _tips_mutex;

  // Handling pinch gesture
  bool _is_pinched;
  int _pinching_hand_id;
  Vec3f _pinch_origin;
  Vec3f _pinch_last_read;
  Vec3f _pinch_last_recorded;
  bool _pin_z;
  bool _pin_xy;
};

#endif
