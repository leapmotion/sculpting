#include "stdafx.h"
#include "OrbiterCamera.h"
#include <cinder/CinderMath.h>
#include "LeapInteraction.h"

using namespace cinder;

const float MIN_CAMERA_DIST = 250.0f;
const float MAX_CAMERA_DIST = 350.0f;

const float MIN_FOV = 50.0f;
const float MAX_FOV = 90.0f;

OrbiterCamera::OrbiterCamera(float initialDistance) :
m_theta(100.0f), 
m_phi(0.0f),
m_wheelZoom(0.0f), 
m_camDist(initialDistance),
m_fov(60.0f)
{
  m_fovModifier.Update(0.0f, 0.0, 0.5f);
}

void OrbiterCamera::SetFovModifier(float mod, double currentTime) {
  m_fovModifier.Update(mod, currentTime, 0.95f);
}

void OrbiterCamera::OnMouseMove(float dX, float dY) {
  m_theta -= dX;
  m_phi += dY;

  if (m_theta<0.f) m_theta += float(M_PI)*2.f;
  if (m_theta >= M_PI*2.f) m_theta -= float(M_PI)*2.f;
  m_phi = math<float>::clamp(m_phi, float(-M_PI)*0.45f, float(M_PI)*0.45f);

  // New camera update.
  util.RecordUserInput(dX, dY, 0.f);
}

void OrbiterCamera::OnResize(float newAspectRatio) {
  setPerspective(80.0f + m_fovModifier.value, newAspectRatio, 1.0f, 100000.f);
}

void OrbiterCamera::Update(float dTheta, float dPhi, float dZoom) {
  m_theta -= dTheta;
  m_phi += dPhi;
  m_fov += dZoom;

  m_fov += m_wheelZoom;
  m_wheelZoom = 0.0f;

  if (m_theta<0.f) m_theta += float(M_PI)*2.f;
  if (m_theta >= M_PI*2.f) m_theta -= float(M_PI)*2.f;
  m_phi = math<float>::clamp(m_phi, float(-M_PI)*0.45f, float(M_PI)*0.45f);
  m_fov = math<float>::clamp(m_fov, 40.f, 110.f);

  const float blend = (m_fov - MIN_FOV) / (MAX_FOV - MIN_FOV);
  m_camDist = blend*(MAX_CAMERA_DIST - MIN_CAMERA_DIST) + MIN_CAMERA_DIST;
}
