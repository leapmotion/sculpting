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

  void setIdentity() {
    translation.setZero();
    rotation.setIdentity();
  }

  void mul(const lmQuat& q) {
    translation = q * translation;
    rotation = rotation * q;
  }

  void transformVec(const Vector3& in, Vector3* out) {
    *out = rotation * in + translation;
  }

  Vector3 operator*(const Vector3& v) { return rotation * v + translation; }
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


#if _WIN32
typedef uint8_t uint8;
typedef uint32_t uint32;
#endif

#endif
