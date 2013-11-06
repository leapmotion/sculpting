#include "StdAfx.h"
#include "Brush.h"

#include <cinder/gl/gl.h>
#include <cinder/Color.h>

Brush::Brush() : _radius(10), _radius_squared(_radius*_radius), _length(20), _strength(1.0f),
  _position(Vector3::Zero()), _direction(Vector3::UnitZ()), _velocity(Vector3::Zero())
{ }

void Brush::reflect(int axis) {
  _position[axis] *= -1;
  _direction[axis] *= -1;
  _velocity[axis] *= -1;
}

Brush Brush::reflected(int axis) const {
  Brush brush(*this);
  brush.reflect(axis);
  return brush;
}

void Brush::transform(const Matrix4x4& matrix) {
  Vector4 temp;
  temp << _position, 1;
  _position = (matrix * temp).head<3>();
  temp << _velocity, 0;
  _velocity = (matrix * temp).head<3>();
  temp << _direction, 0;
  _direction = (matrix * temp).head<3>();
  temp << _worldPos, 1;
  _worldPos = (matrix * temp).head<3>();
}

Brush Brush::transformed(const Matrix4x4& matrix) const {
  Brush brush(*this);
  brush.transform(matrix);
  return brush;
}

void Brush::spinVelocity(const Vector3& rotOrigin, const Vector3& rotAxis, float rotVel) {
  const Vector3 spinVel = rotVel * (_position - rotOrigin).cross(rotAxis);
  _velocity += spinVel;
}

Brush Brush::withSpinVelocity(const Vector3& rotOrigin, const Vector3& rotAxis, float rotVel) const {
  Brush brush(*this);
  brush.spinVelocity(rotOrigin, rotAxis, rotVel);
  return brush;
}

void Brush::draw() const {
#if 1
  ci::gl::drawSphere(ci::Vec3f(_position.x(), _position.y(), _position.z()), _radius, 30);
#else
  Utilities::drawCapsule(_radius, _length, ci::Vec3f(_position.data()), ci::Vec3f(_direction.data()));
#endif
}
