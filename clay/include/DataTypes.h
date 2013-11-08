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

// rotations
typedef Eigen::Quaternion<float> Quaternion; // causes a name clash with Cinder
typedef Eigen::Quaternion<float> lmQuat;
typedef Eigen::AngleAxis<float> AngleAxis;

typedef std::vector<Vector3, Eigen::aligned_allocator<Vector3> > Vector3Vector;

typedef float lmReal;

struct lmTransform {
  Vector3 translation;
  lmQuat rotation;
};

struct Material {
  Material() : ambientFactor(0.0f), diffuseFactor(1.0f), reflectionFactor(0.0f), surfaceColor(Vector3::Ones()),
    reflectionBias(0.0f), refractionBias(0.0f), refractionIndex(0.5f)
  { }
  float ambientFactor;
  float diffuseFactor;
  float reflectionFactor;
  Vector3 surfaceColor;
  float reflectionBias;
  float refractionBias;
  float refractionIndex;
};

#endif
