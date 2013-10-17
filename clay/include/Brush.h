#ifndef __BRUSH_H__
#define __BRUSH_H__

#include "DataTypes.h"
#include <vector>

class Brush {
public:
	float _radius;
	float _radius_squared;
	float _transformed_radius;
	float _transformed_radius_squared;
	float _strength;
	float _weight;
	Vector3 _position;
	Vector3 _transformed_position;
	Vector3 _direction;
  Vector3 _transformed_direction;
  Vector3 _velocity;
  Vector3 _transformed_velocity;
};

typedef std::vector<Vector3, Eigen::aligned_allocator<Vector3> > Vector3Vector;
typedef std::vector<Brush, Eigen::aligned_allocator<Brush> > BrushVector;

#endif
