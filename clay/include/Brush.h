#ifndef __BRUSH_H__
#define __BRUSH_H__

#include "DataTypes.h"
#include "Utilities.h"
#include <vector>

class Brush {
public:
  Brush();

  // point sampling
#if _WIN32
  inline float distance(const Vector3& point) const { return std::sqrtf(distanceSq(point)); }
#else
  inline float distance(const Vector3& point) const { return std::sqrt(distanceSq(point)); }
#endif
#if 1
  inline float distanceSq(const Vector3& point) const { return (point - _position).squaredNorm(); }
#else
  inline float distanceSq(const Vector3& point) const {
    const float t = (point-_position).dot(_direction)/_length;
    Vector3 closest;
    if (t <= 0.0f) {
      closest = _position;
    } else if (t >= 1.0f) {
      closest = _position + _length*_direction;
    } else {
      closest = _position + t*_length*_direction;
    }
    return (point-closest).squaredNorm();
  }
#endif
  inline float strengthAt(const Vector3& point) const { return _strength*Utilities::falloff(distance(point)/_radius); }
  inline Vector3 velocityAt(const Vector3& point) const { return _velocity * strengthAt(point); }
  inline Vector3 pushPullAt(const Vector3& point) const { return _direction * strengthAt(point); }
  inline bool contains(const Vector3& point) const { return distanceSq(point) < _radius_squared; }

  // bounding sphere
  inline Vector3 boundingSphereCenter() const { return (_position + 0.5f*_length*_direction); }
  inline float boundingSphereRadius() const { return (_radius + 0.5f*_length); }
  inline float boundingSphereRadiusSq() const {
    const float radiusSq = boundingSphereRadius();
    return radiusSq*radiusSq;
  }

  // reflection and transformation
  void reflect(int axis);
  Brush reflected(int axis) const;
  void transform(const Matrix4x4& matrix);
  Brush transformed(const Matrix4x4& matrix) const;
  void spinVelocity(const Vector3& rotOrigin, const Vector3& rotAxis, float rotVel);
  Brush withSpinVelocity(const Vector3& rotOrigin, const Vector3& rotAxis, float rotVel) const;

  // drawing
  void draw(float radiusMult) const;

public:
  float _radius;
  float _radius_squared;
  float _length;
  float _strength;
  float _activation;
  Vector3 _position;
  Vector3 _direction;
  Vector3 _velocity;
  Vector3 _worldPos;
};

typedef std::vector<Brush, Eigen::aligned_allocator<Brush> > BrushVector;

#endif
