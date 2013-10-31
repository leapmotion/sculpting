#pragma once
#ifndef __DebugDrawUtil_h__
#define __DebugDrawUtil_h__

#include "DataTypes.h"
#include <vector>


class Mesh;
class Triangle;

// Utility in temp-dev stage. Needed to lock access to bufferes for multithreading.
class DebugDrawUtil {
public:

  struct Buffers {
    // Debug drawing
    std::vector<Vector3> debugPoints;
    std::vector<Vector3> debugLines;
    std::vector<Vector3> debugTriangles;
  };

  DebugDrawUtil() {
    m_readyBufferIdx = 1;
  }

  void DrawCross(const Vector3& position, lmReal size);

  void DrawArrow(const Vector3& from, const Vector3& to);

  void DrawTriangle(const Mesh* mesh, const Triangle& tri);

  void SwitchBuffers();

  void FlushDebugPrimitives();

  std::vector<Vector3>& GetDebugPoints() { TODO(adrian, not thread safe); Buffers& b = m_buffers[(1+m_readyBufferIdx)%2]; return b.debugPoints; }
  std::vector<Vector3>& GetDebugLines() { TODO(adrian, not thread safe); Buffers& b = m_buffers[(1+m_readyBufferIdx)%2]; return b.debugLines; }
  std::vector<Vector3>& GetDebugTriangles() { TODO(adrian, not thread safe); Buffers& b = m_buffers[(1+m_readyBufferIdx)%2]; return b.debugTriangles; }

  Buffers m_buffers[2];
  int m_readyBufferIdx;

  boost::mutex m_mutex;
};



#endif // __DebugDrawUtil_h__
