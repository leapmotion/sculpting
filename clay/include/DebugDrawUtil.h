#pragma once
#ifndef __DebugDrawUtil_h__
#define __DebugDrawUtil_h__

#include "DataTypes.h"
#include <vector>



// Debug drawing
extern std::vector<Vector3> debugPoints;
extern std::vector<Vector3> debugLines;
extern std::vector<Vector3> debugTriangles;

// Utility in temp-dev stage. Needed to lock access to bufferes for multithreading.
class DebugDrawUtil {
public:

  void DrawCross(const Vector3& position, lmReal size);

  void DrawArrow(const Vector3& from, const Vector3& to);

  void FlushDebugPrimitives();

  std::vector<Vector3>& GetDebugPoints() { return debugPoints; }
  std::vector<Vector3>& GetDebugLines() { return debugLines; }
  std::vector<Vector3>& GetDebugTriangles() { return debugTriangles; }

  boost::mutex m_mutex;
};



#endif // __DebugDrawUtil_h__
