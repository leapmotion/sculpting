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
class HandInfo;

class LeapInteraction
{

public:

  LeapInteraction(Sculpt* sculpt, UserInterface* ui);
  bool processInteraction(LeapListener& listener, float aspect, const Matrix44f& modelView, const Matrix44f& projection, const Vec2i& viewport, float referenceDistance, float fov, bool suppress);

  float getDThetaVel() const { return m_cameraX.value; }
  float getDPhiVel() const { return m_cameraY.value; }
  float getDZoomVel() const { return m_cameraZ.value; }
  float getScaleFactor() const { return m_cameraW.value; }

  Vec4f getDeltaVector() const { 
    return Vec4f(m_cameraX.value, m_cameraY.value, m_cameraZ.value, m_cameraW.value);
  }

  Vec3f getPinchDeltaFromLastCall();
  bool isPinched() const { return _is_pinched; }
  void setBrushRadius(float _Radius) { _desired_brush_radius = _Radius; }
  void setBrushStrength(float _Strength) { _desired_brush_strength = _Strength; }
  void setBrushAuto(bool autoBrush) { _autoBrush = autoBrush; }
  double mostRecentTime() const { return Utilities::TIME_STAMP_TICKS_TO_SECS*static_cast<double>(_cur_frame.timestamp()); }
  std::vector<Vec4f> getTips() { std::unique_lock<std::mutex> tipsLock(_tips_mutex); return _tips; }
  double getLastCameraUpdateTime() const { return _last_camera_update_time; }
  double getLastActivityTime() const { return _last_activity_time; }

  static const float MIN_POINTABLE_LENGTH;
  static const float MIN_POINTABLE_AGE;

private:

  void interact(double curTime);
  void updateHandInfos(double curTime);
  void cleanUpHandInfos(double curTime);

  static bool paddleTranslation(const Leap::Hand& hand, const Leap::Frame& sinceFrame, Leap::Vector& trans) {
    if (fabs(hand.palmNormal().y) < 0.5f || hand.pointables().count() > 2) {
      const Leap::Vector translation = hand.translation(sinceFrame);
#if 0
      float mult = std::fabs(hand.palmNormal().dot(translation.normalized()));
      trans = mult*mult*translation;
#else
      trans = translation;
#endif
      return true;
    } else {
      return false;
    }
  }

  Vector3 calcSize(float fov, float referenceDistance) {
    const float width = 2.0f * referenceDistance * tan(fov/2.0f);
    const float height = width;
    const float depth = 2.0f * referenceDistance;
    return Vector3(width, height, depth);
  }

  Leap::Frame _cur_frame;
  Leap::Frame _last_frame;

  Sculpt* _sculpt;
  UserInterface* _ui;

  std::mutex _tips_mutex;
  std::vector<Vec4f> _tips;
  Matrix44f _model_view_inv;
  Matrix44f _model_view;
  Matrix44f _projection;
  Vec2i _window_size;
  float _desired_brush_radius;
  float _desired_brush_strength;


  Utilities::ExponentialFilter<float> m_cameraX; //theta
  Utilities::ExponentialFilter<float> m_cameraY; //phi
  Utilities::ExponentialFilter<float> m_cameraZ; //zoom
  Utilities::ExponentialFilter<float> m_cameraW; //scale

  double _last_camera_update_time;
  float _reference_distance;
  float _fov;
  bool _autoBrush;
  double _last_activity_time;

  // Handling pinch gesture
  bool _is_pinched;
  int _pinching_hand_id;
  Vec3f _pinch_origin;
  Vec3f _pinch_last_read;
  Vec3f _pinch_last_recorded;
  bool _pin_z;
  bool _pin_xy;

#if _WIN32
  typedef std::map<int, HandInfo, std::less<int>, Eigen::aligned_allocator< std::pair<const int, HandInfo> > > HandInfoMap;
#else
  typedef std::map<int, HandInfo> HandInfoMap;
#endif
  
  HandInfoMap _hand_infos;
};


#endif
