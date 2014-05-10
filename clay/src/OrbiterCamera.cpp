#include "stdafx.h"
#include "OrbiterCamera.h"
#include <cinder/CinderMath.h>
#include "LeapInteraction.h"

using namespace cinder;

OrbiterCamera::OrbiterCamera() :
mWheelZoom(0.0f),
mFocusPoint(Vector3::Zero())
{
  mFovModifier.Update(0.0f, 0.0, 0.5f);
}

void OrbiterCamera::setFovModifier(float mod, double currentTime) {
  mFovModifier.Update(mod, currentTime, 0.95f);
}

void OrbiterCamera::onMouseMove(float dX, float dY) {
  // New camera update.
  util.RecordUserInput(dX, dY, 0.f);
}

void OrbiterCamera::onResize(float newAspectRatio) {
  setPerspective(80.0f + mFovModifier.value, newAspectRatio, 1.0f, 100000.f);
}

void OrbiterCamera::update(const Vec4f &deltaVector, double curTime, Mesh* mesh) {
  util.RecordUserInput(deltaVector.x, deltaVector.y, deltaVector.z + mWheelZoom);

  mWheelZoom = 0.0f;

  lmTransform tCamera = util.GetCameraInWorldSpace();

  Vec3f campos = ToVec3f(tCamera.translation);
  Vector3 up = tCamera.rotation * Vector3::UnitY();
  Vector3 to = tCamera.translation + tCamera.rotation * Vector3::UnitZ() * -200.0f;

  {
    Matrix4x4 trans;
    if (mesh)
      trans = mesh->getTransformation();
    else
      trans = Matrix4x4::Identity();

    Vector4 temp = util.GetIsoStateReferencePosition();
    mFocusPoint = (trans * temp).head<3>();
  }
  
  // if mesh
  if (mesh)
    mFocusRadius = util.IsoQueryRadius(mesh);
  else
    mFocusRadius = 0.0f;

  mPositionSmoother.Update(campos, curTime, 0.95f);
  mLookAtSmoother.Update(ToVec3f(to), curTime, 0.95f);
  mUpSmoother.Update(ToVec3f(up), curTime, 0.95f);

  lookAt(mPositionSmoother.value, mLookAtSmoother.value, mUpSmoother.value.normalized());
  getProjectionMatrix();
  
}
