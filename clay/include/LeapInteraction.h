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

  float getDThetaVel() const { return _dtheta.value; }
  float getDPhiVel() const { return _dphi.value; }
  float getDZoomVel() const { return _dzoom.value; }
  float getScaleFactor() const { return _scaleFactor.value; }

  Vec4f getDeltaVector() const { return Vec4f(_dtheta.value, _dphi.value, _dzoom.value, _scaleFactor.value); }

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
  std::vector<Vec4f> _tips;
  Matrix44f _model_view_inv;
  Matrix44f _model_view;
  Matrix44f _projection;
  Vec2i _window_size;
  float _desired_brush_radius;
  float _desired_brush_strength;
  Utilities::ExponentialFilter<float> _dphi;
  Utilities::ExponentialFilter<float> _dtheta;
  Utilities::ExponentialFilter<float> _dzoom;
  Utilities::ExponentialFilter<float> _scaleFactor;
  std::mutex _tips_mutex;
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

class HandInfo {
public:

  HandInfo() : m_lastUpdateTime(0.0), m_lastHandOpenChangeTime(0.0), m_handOpen(false), m_firstUpdate(true), m_lastPalmPos(Vector3::Zero()) {
    m_translation.value = Vector3::Zero();
    m_transRatio.value = 0.5f;
    m_normalY.value = 0.5f;
  }

  int getNumFingers() const { return m_numFingers.FilteredCategory(); }
  Vector3 getTranslation() const { return m_translation.value; }
  float getTranslationRatio() const { return m_transRatio.value; }
  float getNormalY() const { return m_normalY.value; }
  double getLastUpdateTime() const { return m_lastUpdateTime; }
  double getLastHandOpenChangeTime() const { return m_lastHandOpenChangeTime; }
  bool handOpen() const { return m_handOpen; }

  Vector3 getModifiedTranslation() const {
    const float ratio = getTranslationRatio();
    const Vector3 origTrans = getTranslation();
    return Vector3(ratio*origTrans.x(), ratio*origTrans.y(), (1.0f-ratio)*origTrans.z());
  }

  void update(const Leap::Hand& hand, const Leap::Frame& sinceFrame, double curTime) {
    static const float NUM_FINGERS_SMOOTH_STRENGTH = 0.9f;
    static const float TRANSLATION_SMOOTH_STRENGTH = 0.5f;
    static const float TRANSLATION_RATIO_SMOOTH_STRENGTH = 0.985f;
    static const float NORMAL_Y_SMOOTH_STRENGTH = 0.9f;
    m_lastUpdateTime = curTime;

    // update number of fingers
    const Leap::PointableList pointables = hand.pointables();
    int numFingers = 0;
    for (int i=0; i<pointables.count(); i++) {
      if (pointables[i].length() >= LeapInteraction::MIN_POINTABLE_LENGTH && pointables[i].timeVisible() >= LeapInteraction::MIN_POINTABLE_AGE) {
        numFingers++;
      }
    }
    m_numFingers.Update(numFingers, curTime, NUM_FINGERS_SMOOTH_STRENGTH);
    const int curNumFingers = m_numFingers.FilteredCategory();
    if (curNumFingers > 2 && !m_handOpen) {
      m_handOpen = true;
      m_lastHandOpenChangeTime = curTime;
    } else if (m_handOpen && curNumFingers <= 2) {
      m_handOpen = false;
      m_lastHandOpenChangeTime = curTime;
    }

    // update translation
    const Leap::Vector temp(hand.palmPosition());
    const Vector3 curPos(temp.x, temp.y, temp.z);
    Vector3 translation = curPos - m_lastPalmPos;
    if (m_firstUpdate) {
      translation = Vector3::Zero();
    }
    m_lastPalmPos = curPos;
    m_firstUpdate = false;
    m_translation.Update(translation, curTime, TRANSLATION_SMOOTH_STRENGTH);

    // update translation ratio
    if (translation.squaredNorm() > 0.001f) {
      const Vector3 unitTranslation = translation.normalized();
      const float curRatio = unitTranslation.x()*unitTranslation.x() + unitTranslation.y()*unitTranslation.y();
      m_transRatio.Update(curRatio, curTime, TRANSLATION_RATIO_SMOOTH_STRENGTH);
    }

    // update palm normal Y value
    m_normalY.Update(fabs(hand.palmNormal().y), curTime, NORMAL_Y_SMOOTH_STRENGTH);
  }
private:
  bool m_firstUpdate;
  Vector3 m_lastPalmPos;
  Utilities::CategoricalFilter<10> m_numFingers;
  Utilities::ExponentialFilter<Vector3> m_translation;
  Utilities::ExponentialFilter<float> m_transRatio;
  Utilities::ExponentialFilter<float> m_normalY;
  double m_lastUpdateTime;
  bool m_handOpen;
  double m_lastHandOpenChangeTime;
};

#endif
