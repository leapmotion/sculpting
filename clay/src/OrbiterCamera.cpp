#include "stdafx.h"
#include "OrbiterCamera.h"
#include <cinder/CinderMath.h>
#include "LeapInteraction.h"

using namespace cinder;

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

void OrbiterCamera::update(const Vec4f &deltaVector) {
  util.RecordUserInput(deltaVector.x, deltaVector.y, deltaVector.z + mWheelZoom);

  mWheelZoom = 0.0f;
}
