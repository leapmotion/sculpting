#ifndef __DataTypes_h__
#define __DataTypes_h__

#include <Eigen/Dense>

// matrices
typedef Eigen::Matrix<float, 2, 2> Matrix2x2;
typedef Eigen::Matrix<float, 3, 3> Matrix3x3;
typedef Eigen::Matrix<float, 4, 4> Matrix4x4;

// vectors
typedef Eigen::Matrix<float, 2, 1> Vector2;
typedef Eigen::Matrix<float, 3, 1> Vector3;
typedef Eigen::Matrix<float, 4, 1> Vector4;

#endif
