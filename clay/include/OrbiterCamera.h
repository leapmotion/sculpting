#pragma once

#include "cinder/Camera.h"
#include "Utilities.h"
#include "CameraUtil.h"
#include "DataTypes.h"
#include "Mesh.h"

//This is a cinder derived class, so as long as it is, we'll use cinder-style naming conventions

class OrbiterCamera : public cinder::CameraPersp
{
public:
  OrbiterCamera();

  void onMouseMove(float dX, float dY);
  void onMouseWheel(float zoom) { mWheelZoom = -300.0f * zoom; }

  void update(const Vec4f &deltaVector, double curTime, Mesh* mesh);

  void onResize(float newAspectRatio);

  Vector3 getFocusPoint() const { return mFocusPoint; }
  float getFocusRadius() const { return mFocusRadius; }

  CameraUtil util;
private:
  Utilities::ExponentialFilter<float> mFovModifier;
  float mWheelZoom;

  Vector3 mFocusPoint;
  float mFocusRadius; //possibly Conflated with brush size

  Utilities::ExponentialFilter<ci::Vec3f> mPositionSmoother;
  Utilities::ExponentialFilter<ci::Vec3f> mLookAtSmoother;
  Utilities::ExponentialFilter<ci::Vec3f> mUpSmoother;
};