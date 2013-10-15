#ifndef __DataTypes_h__
#define __DataTypes_h__

#include <Eigen/Dense>
#include <vector>

// matrices
typedef Eigen::Matrix<float, 2, 2> Matrix2x2;
typedef Eigen::Matrix<float, 3, 3> Matrix3x3;
typedef Eigen::Matrix<float, 4, 4> Matrix4x4;

// vectors
typedef Eigen::Matrix<float, 2, 1> Vector2;
typedef Eigen::Matrix<float, 3, 1> Vector3;
typedef Eigen::Matrix<float, 4, 1> Vector4;

// color
class QColor {
public:
	QColor(int r, int g, int b, int a = 255) : _color(r/255.0f, g/255.0f, b/255.0f, a/255.0f) { }
	QColor(float r, float g, float b, float a = 1.0f) : _color(r, g, b, a) { }
	float red() const { return _color.x(); }
	float green() const { return _color.y(); }
	float blue() const { return _color.z(); }
	float alpha() const { return _color.w(); }
private:
	Vector4 _color;
};

struct Brush {
	float _radius;
	float _radius_squared;
	float _transformed_radius;
	float _transformed_radius_squared;
	float _strength;
	float _weight;
	Vector3 _position;
	Vector3 _transformed_position;
	Vector3 _direction;
  Vector3 _velocity;
  Vector3 _transformed_velocity;
};

typedef std::vector<Vector3, Eigen::aligned_allocator<Vector3> > Vector3Vector;
typedef std::vector<Brush, Eigen::aligned_allocator<Brush> > BrushVector;

// rotations
typedef Eigen::Quaternion<float> Quaternion;
typedef Eigen::AngleAxis<float> AngleAxis;

#endif
