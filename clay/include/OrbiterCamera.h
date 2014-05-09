#pragma once

#include "cinder/Camera.h"
#include "Utilities.h"
#include "CameraUtil.h"
#include "DataTypes.h"

//This is a cinder derived class, so as long as it is, we'll use cinder-style naming conventions

class OrbiterCamera : public cinder::CameraPersp
{
public:
  OrbiterCamera();

  void onMouseMove(float dX, float dY);
  void onMouseWheel(float zoom) { mWheelZoom = -300.0f * zoom; }

  void update(const Vec4f &deltaVector);

  void onResize(float newAspectRatio);

  CameraUtil util;
private:
  Utilities::ExponentialFilter<float> mFovModifier;
  float mWheelZoom;
};