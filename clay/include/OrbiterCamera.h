#pragma once

#include "cinder/Camera.h"
#include "Utilities.h"
#include "CameraUtil.h"

//This is a cinder derived class, so as long as it is, we'll use cinder-style naming conventions

class OrbiterCamera : public cinder::CameraPersp
{
public:
  OrbiterCamera(float initialDistance);

  void onMouseMove(float dX, float dY);

  void setZoom(float zoom) { mWheelZoom = -300.0f * zoom; }
  void setFovModifier(float mod, double currentTime);

  void update(float dTheta, float dPhi, float dZoom, float curTime, float lastSculptTime );

  void onResize(float newAspectRatio);

  float getZoom() const { return mZoom; };

  CameraUtil util;
private:
  float mTheta;
  float mPhi;
  float mZoom;
  float mCamDist;
  Utilities::ExponentialFilter<float> mFovModifier;
  float mWheelZoom;
};