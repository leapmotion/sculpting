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
  temp << _position, 0;
  _position = (matrix * temp).head<3>();
  //Vector3 modelVel = (_transformed_position - (prevTransform_ * temp).head<3>())/deltaTime;
  //_transformed_radius_squared = _transformed_radius*_transformed_radius;/
  temp << _velocity, 0;
  _velocity = (matrix * temp).head<3>();// + 2.0f*modelVel;
  temp << _direction, 0;
  _direction = (matrix * temp).head<3>();

}

Brush Brush::transformed(const Matrix4x4& matrix) const {
  Brush brush(*this);
  brush.transform(matrix);
  return brush;
}

void Brush::draw() const {
#if 1
	ci::gl::drawSphere(ci::Vec3f(_position.x(), _position.y(), _position.z()), _radius, 30);
#else
  Utilities::drawCapsule(_radius, _length, ci::Vec3f(_position.data()), ci::Vec3f(_direction.data()));
#endif
}
