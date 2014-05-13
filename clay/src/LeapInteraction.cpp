#include "StdAfx.h"
#include "LeapInteraction.h"
#include "cinder/app/AppBasic.h"
#include "Utilities.h"
#include <vector>

#include "ReplayUtil.h"

using namespace ci;

#define USE_SKELETON_API 0

const float LeapInteraction::MIN_POINTABLE_LENGTH = 10.0f;
const float LeapInteraction::MIN_POINTABLE_AGE = 0.05f;



//Temporally consistent, outlier robust hand tracking.  If this is still required by V2,
//We've failed.
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
    return Vector3(ratio*origTrans.x(), ratio*origTrans.y(), (1.0f - ratio)*origTrans.z());
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
    for (int i = 0; i<pointables.count(); i++) {
      if (pointables[i].length() >= LeapInteraction::MIN_POINTABLE_LENGTH && pointables[i].timeVisible() >= LeapInteraction::MIN_POINTABLE_AGE) {
        numFingers++;
      }
    }
    m_numFingers.Update(numFingers, curTime, NUM_FINGERS_SMOOTH_STRENGTH);
    const int curNumFingers = m_numFingers.FilteredCategory();
    if (curNumFingers > 2 && !m_handOpen) {
      m_handOpen = true;
      m_lastHandOpenChangeTime = curTime;
    }
    else if (m_handOpen && curNumFingers <= 2) {
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

LeapInteraction::LeapInteraction(Sculpt* sculpt, UserInterface* ui) : _sculpt(sculpt), _ui(ui),
  _desired_brush_radius(0.4f), _is_pinched(false), _last_camera_update_time(0.0), _autoBrush(true),
  _last_activity_time(0.0)
{
  m_cameraY.Update(0.0f, 0.0, 0.95f);
  m_cameraX.Update(0.0f, 0.0, 0.95f);
  m_cameraZ.Update(0.0f, 0.0, 0.95f);
  m_cameraW.Update(1.0f, 0.0, 0.95f);
}

bool LeapInteraction::processInteraction(LeapListener& listener, float aspect, const Matrix44f& modelView, const Matrix44f& projection, const Vec2i& viewport, float referenceDistance, float fov, bool suppress)
{
  LM_ASSERT_IDENTICAL(124);
  LM_ASSERT_IDENTICAL(aspect);
  LM_ASSERT_IDENTICAL(referenceDistance);
  LM_ASSERT_IDENTICAL(suppress);

  static const double MIN_TIME_BETWEEN_FRAMES = 0.000001;
  _model_view_inv = modelView.inverted();
  _model_view = modelView;
  _projection = projection;
  _window_size = viewport;
  _reference_distance = referenceDistance;
  _fov = fov;
  if (suppress || !LM_RETURN_TRACKED(listener.isConnected()) || !listener.isReceivingFrames())
  {
    _cur_frame = Leap::Frame::invalid();
    _last_frame = Leap::Frame::invalid();
    std::unique_lock<std::mutex> brushLock(_sculpt->getBrushMutex());
    std::unique_lock<std::mutex> tipsLock(_tips_mutex);
    _sculpt->clearBrushes();
    _tips.clear();
    m_cameraX.value = 0.0f;
    m_cameraY.value = 0.0f;
    m_cameraZ.value = 0.0f;
    m_cameraW.value = 1.0f;
  }
  else if (LM_RETURN_TRACKED(listener.waitForFrame(_cur_frame, 33)))
  {
    std::unique_lock<std::mutex> brushLock(_sculpt->getBrushMutex());
    std::unique_lock<std::mutex> tipsLock(_tips_mutex);
    const double time = LM_RETURN_TRACKED(Utilities::TIME_STAMP_TICKS_TO_SECS*static_cast<double>(_cur_frame.timestamp()));
    const double prevTime = LM_RETURN_TRACKED(Utilities::TIME_STAMP_TICKS_TO_SECS*static_cast<double>(_last_frame.timestamp()));
    if (LM_RETURN_TRACKED(_last_frame.isValid() && _cur_frame.isValid()) && time - prevTime > MIN_TIME_BETWEEN_FRAMES) {
      _sculpt->clearBrushes();
      _tips.clear();
      updateHandInfos(time);
      cleanUpHandInfos(time);
      interact(time);
    }
    _last_frame = _cur_frame;
    return true;
  }
  return false;
}

void LeapInteraction::interact(double curTime)
{
  LM_ASSERT_IDENTICAL(124324);
  LM_TRACK_VALUE(curTime);
 
  Vector3 cameraMovement;
  static const float ORBIT_SPEED = 0.01f;
  static const float ZOOM_SPEED = 75.0f;
  static const float AGE_WARMUP_TIME = 0.75f;
  static const float TARGET_DELTA_TIME = 1.0f / 60.0f;
  static const float SCALE_FACTOR_SMOOTH_STRENGTH = 0.5f;

  // create brushes
  static const Vec3f LEAP_OFFSET(0, 250, 100);
  static const Vec3f LEAP_SIZE(275, 275, 275);
  const float ui_mult = 1.0f - _ui->maxActivation();
  const float deltaTime = static_cast<float>(Utilities::TIME_STAMP_TICKS_TO_SECS*(_cur_frame.timestamp() - _last_frame.timestamp()));
  LM_TRACK_CONST_VALUE(deltaTime);
  const float dtMult = deltaTime / TARGET_DELTA_TIME;
  const Vector3 scaledSize = calcSize(_fov, _reference_distance);
  const float frameScale = _cur_frame.scaleFactor(_last_frame);
  LM_TRACK_CONST_VALUE(frameScale);

  if (!_cur_frame.hands().isEmpty() || !_cur_frame.pointables().isEmpty()) {
    _last_activity_time = ci::app::getElapsedSeconds();
  }
  LM_TRACK_VALUE(_last_activity_time);

  int numOpenHands = 0;
  for (HandInfoMap::iterator it = _hand_infos.begin(); it != _hand_infos.end(); ++it) {
    const HandInfo& cur = it->second;
    if (cur.getLastUpdateTime() < curTime) {
      continue;
    }
    const int numFingers = cur.getNumFingers();
    const float normalY = cur.getNormalY();
    if (numFingers > 2 || normalY < 0.35f) {
      numOpenHands++;
    }
  }
  LM_TRACK_VALUE(numOpenHands);

  if (numOpenHands >= 2) {
    m_cameraW.Update(frameScale, curTime, SCALE_FACTOR_SMOOTH_STRENGTH);
  } else {
    m_cameraW.Update(1.0f, curTime, SCALE_FACTOR_SMOOTH_STRENGTH);
    for (HandInfoMap::iterator it = _hand_infos.begin(); it != _hand_infos.end(); ++it) {
      LM_ASSERT_IDENTICAL(0x12345678);
      const int id = it->first;
      const HandInfo& cur = it->second;
      const float normalY = LM_RETURN_TRACKED(cur.getNormalY());
      if (LM_RETURN_TRACKED(cur.getLastUpdateTime() < curTime)) {
        continue;
      }
      const float timeSinceHandOpenChange = static_cast<float>(curTime - cur.getLastHandOpenChangeTime());
      if (cur.handOpen() || normalY < 0.1f) {
        // camera interaction
        cameraMovement = LM_RETURN_TRACKED(cur.getModifiedTranslation());
        cameraMovement *= ORBIT_SPEED;
        cameraMovement.y() = -cameraMovement.y();
        cameraMovement.z() = -cameraMovement.z();
        _last_camera_update_time = ci::app::getElapsedSeconds();
      } else {
        // sculpting interaction
        const Leap::Hand hand = _cur_frame.hand(id);
        if (LM_RETURN_TRACKED(hand.isValid())) {
          const Leap::Pointable pointable = hand.pointables().frontmost();
          if (LM_RETURN_TRACKED(pointable.isValid()) &&
              LM_RETURN_TRACKED(pointable.length()) >= MIN_POINTABLE_LENGTH &&
              LM_RETURN_TRACKED(pointable.timeVisible()) >= MIN_POINTABLE_AGE) {
            // add brush
            const float strengthMult = Utilities::SmootherStep(math<float>::clamp(LM_RETURN_TRACKED(std::min(timeSinceHandOpenChange,pointable.timeVisible()))/AGE_WARMUP_TIME));

            Leap::Vector tip_pos = LM_RETURN_TRACKED(pointable.tipPosition());
            Leap::Vector tip_dir = LM_RETURN_TRACKED(pointable.direction());
            Leap::Vector tip_vel = LM_RETURN_TRACKED(pointable.tipVelocity());

            Vec3f pos = Vec3f(tip_pos.x, tip_pos.y, tip_pos.z) - LEAP_OFFSET;
            Vec3f dir = Vec3f(tip_dir.x, tip_dir.y, tip_dir.z);
            Vec3f vel = Vec3f(tip_vel.x, tip_vel.y, tip_vel.z);

            float ratioX = pos.x / LEAP_SIZE.x;
            float ratioY = pos.y / LEAP_SIZE.y;
            float ratioZ = pos.z / LEAP_SIZE.z;

            pos.x = ratioX * scaledSize.x();
            pos.y = ratioY * scaledSize.y();
            pos.z = ratioZ * scaledSize.z();

            Vector3 brushPos(_model_view_inv.transformPoint(pos).ptr());
            Vector3 brushDir((-_model_view_inv.transformVec(dir)).ptr());
            Vector3 brushVel(_model_view_inv.transformVec(vel).ptr());

            const float fromCameraMult = ci::math<float>::clamp((LEAP_OFFSET.z/2.0f - tip_pos.z)/50.0f);

            float strength = strengthMult*ui_mult*_desired_brush_strength;
            strength = std::min(1.0f, strength * dtMult);
            LM_TRACK_VALUE(strength);

            Vec3f transPos = _projection.transformPoint(pos);
            Vec3f radPos = _projection.transformPoint(pos+Vec3f(_desired_brush_radius, 0, 0));
            LM_TRACK_VALUE(transPos);
            LM_TRACK_VALUE(radPos);

            // compute screen-space coordinate of this finger
            transPos.x = (transPos.x + 1)/2;
            transPos.y = (transPos.y + 1)/2;
            transPos.z = 1.0f;

            const float autoBrushScaleFactor = (scaledSize.x() / LEAP_SIZE.x);
            float adjRadius = _desired_brush_radius;
            if (_autoBrush) {
              adjRadius *= autoBrushScaleFactor;
            }
            static const float BORDER_THICKNESS = 0.035f;
            if (transPos.x >= -BORDER_THICKNESS && transPos.x <= (1.0f + BORDER_THICKNESS) && transPos.y >= -BORDER_THICKNESS && transPos.y <= (1.0f + BORDER_THICKNESS)) {
              // compute a point on the surface of the sphere to use as the screen-space radius
              radPos.x = (radPos.x + 1)/2;
              radPos.y = (radPos.y + 1)/2;
              radPos.z = 1.0f;
              float rad = transPos.distance(radPos);
              if (_autoBrush) {
                rad *= autoBrushScaleFactor;
              }
              transPos.z = rad;
              Vec4f tip(transPos.x, transPos.y, transPos.z, fromCameraMult*strengthMult);
              LM_ASSERT_IDENTICAL(tip);
              _tips.push_back(tip);
            } else {
              strength = 0.0f;
            }
            if (strengthMult > 0.25f && ui_mult > 0.25f) {
              _sculpt->addBrush(Vector3(pos.ptr()), brushPos, brushDir, brushVel, adjRadius, strength, fromCameraMult*strengthMult);
            }
          }
        }
      }
    }
  }

  cameraMovement /= deltaTime;

  static const float SMOOTH_STRENGTH = 0.4f;
  m_cameraX.Update(cameraMovement.x(), curTime, SMOOTH_STRENGTH);
  m_cameraY.Update(cameraMovement.y(), curTime, SMOOTH_STRENGTH);
  m_cameraZ.Update(cameraMovement.z(), curTime, SMOOTH_STRENGTH);

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

void LeapInteraction::updateHandInfos(double curTime) {
  LM_ASSERT_IDENTICAL(123454);
  const Leap::HandList hands = _cur_frame.hands();
  if (!LM_TRACK_IS_REPLAYING()) {
    for (int i=0; i<hands.count(); i++) {
      int id = hands[i].id();
      _hand_infos[id].update(hands[i], _last_frame, curTime);
    }
  }

  int numHands = LM_RETURN_TRACKED(hands.count());

  { // tracking code only:
    for (int i = 0; i < numHands; i++)
    {
      int id = LM_RETURN_TRACKED_CONDITIONAL(i < hands.count(), hands[i].id(), int);
      _hand_infos[id] = LM_RETURN_TRACKED_CONDITIONAL(i < hands.count(), _hand_infos[id], HandInfo);
    }
  }
}

void LeapInteraction::cleanUpHandInfos(double curTime) {
  LM_ASSERT_IDENTICAL(122354);
  LM_ASSERT_IDENTICAL(curTime);
  LM_ASSERT_IDENTICAL(_hand_infos.size());
  static const float MAX_HAND_INFO_AGE = 0.1f; // seconds since last update until hand info gets cleaned up
  HandInfoMap::iterator it = _hand_infos.begin();
  while (it != _hand_infos.end()) {
    HandInfo& cur = it->second;
    const float curAge = fabs(static_cast<float>(curTime - cur.getLastUpdateTime()));
    if (curAge > MAX_HAND_INFO_AGE) {
      _hand_infos.erase(it++);
    } else {
      ++it;
    }
  }
  LM_ASSERT_IDENTICAL(_hand_infos.size());
}

