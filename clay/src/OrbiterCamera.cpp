#include "stdafx.h"
#include "OrbiterCamera.h"
#include <cinder/CinderMath.h>
#include "LeapInteraction.h"

using namespace cinder;

const float MIN_CAMERA_DIST = 250.0f;
const float MAX_CAMERA_DIST = 350.0f;

const float MIN_FOV = 50.0f;
const float MAX_FOV = 90.0f;

OrbiterCamera::OrbiterCamera() :
mWheelZoom(0.0f)
{
}

void OrbiterCamera::onMouseMove(float dX, float dY) {
  // New camera update.
  util.RecordUserInput(dX, dY, 0.f);
}

void OrbiterCamera::onResize(float newAspectRatio) {
  setPerspective(80.0f, newAspectRatio, 1.0f, 100000.f);
}

void OrbiterCamera::update(const Vec4f &deltaVector, float curTime, float lastSculptTime) {
  const float sculptMult = std::min(1.0f, static_cast<float>(fabs(curTime - lastSculptTime)) / 0.5f);
  const Vec4f sculptVec = (deltaVector + Vec4f(0,0,mWheelZoom,0) )*sculptMult;

  util.RecordUserInput(sculptVec.x, sculptVec.y, sculptVec.z);

  mWheelZoom = 0.0f;
}
