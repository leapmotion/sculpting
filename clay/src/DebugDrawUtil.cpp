#include "StdAfx.h"
#include "DebugDrawUtil.h"




// Helper func

void DebugDrawUtil::DrawCross(const Vector3& position, lmReal size) {
  Buffers& b = m_buffers[(1+m_readyBufferIdx)%2];
  b.debugLines.push_back(Vector3(position + Vector3::UnitX() * size));
  b.debugLines.push_back(Vector3(position - Vector3::UnitX() * size));
  b.debugLines.push_back(Vector3(position + Vector3::UnitY() * size));
  b.debugLines.push_back(Vector3(position - Vector3::UnitY() * size));
  b.debugLines.push_back(Vector3(position + Vector3::UnitZ() * size));
  b.debugLines.push_back(Vector3(position - Vector3::UnitZ() * size));
}

void DebugDrawUtil::DrawArrow(const Vector3& from, const Vector3& to) {
  Buffers& b = m_buffers[(1+m_readyBufferIdx)%2];
  b.debugLines.push_back(from);
  b.debugLines.push_back(to);
  // todo: draw arrow head
}

void DebugDrawUtil::SwitchBuffers() {
  boost::unique_lock<boost::mutex> lock(m_mutex);
  m_readyBufferIdx = 1 - m_readyBufferIdx;

  Buffers& b = m_buffers[(1+m_readyBufferIdx)%2];
  b.debugLines.clear();
  b.debugPoints.clear();
  b.debugTriangles.clear();
}

void DebugDrawUtil::FlushDebugPrimitives() {
  boost::unique_lock<boost::mutex> lock(m_mutex);
  const int primNames[] = { GL_POINTS, GL_LINES, GL_TRIANGLES };
  Buffers& b = m_buffers[m_readyBufferIdx];
  std::vector<Vector3>* primBuffers[] = { &b.debugPoints, &b.debugLines, &b.debugTriangles };
  glDisable(GL_DEPTH_TEST);
  glLineWidth(2.0f);
  glPolygonMode(GL_FRONT, GL_LINE);
  for (int pi = 0; pi < 3; pi++) {
    // Display debug buffers
    glBegin(primNames[pi]);
    std::vector<Vector3>& buffer = *primBuffers[pi];
    for (size_t i = 0; i < buffer.size(); i++) {
      const Vector3& v = buffer[i];
      glVertex3f(v.x(), v.y(), v.z());
    }
    glEnd();
    // Clear debug buffers
    //buffer.clear();
  }
  glPolygonMode(GL_FRONT, GL_FILL);
  glEnable(GL_DEPTH_TEST);
}

