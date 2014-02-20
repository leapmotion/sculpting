#ifndef __TOOLS_H__
#define __TOOLS_H__

#include "DataTypes.h"
#include <vector>
#include <algorithm>
#include <iostream>

/**
* Tools
* @author Stéphane GINIER
*/
namespace Tools
{
  /** Sort and delete duplicate values */
  template<typename T> void tidy(std::vector<T> &vec)
  {
    std::sort(vec.begin(), vec.end());
    vec.resize(std::unique(vec.begin(), vec.end())-vec.begin());
  }

  void coutMatrix(const Matrix4x4& matrix);
  void coutVertex(const Vector3& vertex);

  Matrix4x4 translationMatrix(const Vector3& translation);
  Matrix4x4 scaleMatrix(float scale);
  Matrix4x4 rotationMatrix(const Vector3& axis, float angle);
}

#endif /*__TOOLS_H__*/
