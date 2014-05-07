#pragma once

#include "cinder/Camera.h"
#include "Utilities.h"

class OrbiterCamera : public cinder::CameraPersp
{
public:
  OrbiterCamera(float initialDistance);
  void OnMouseMove(float dX, float dY);

  void SetZoom(float zoom) { m_wheelZoom = -300.0f * zoom; }
  void SetFovModifier(float mod, double currentTime);

  void Update(float dTheta, float dPhi, float dZoom);

  void OnResize(float newAspectRatio);

  float GetFov() const { return m_fov; };

private:
  float m_theta;
  float m_phi;
  float m_fov;
  float m_camDist;
  Utilities::ExponentialFilter<float> m_fovModifier;
  float m_wheelZoom;
};