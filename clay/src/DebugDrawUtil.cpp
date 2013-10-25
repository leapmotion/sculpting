#include "StdAfx.h"
#include "DebugDrawUtil.h"


// Debug output
std::vector<Vector3> debugPoints;
std::vector<Vector3> debugLines;
std::vector<Vector3> debugTriangles;


// Helper func

void DebugDrawUtil::DrawCross(const Vector3& position, lmReal size) {
  boost::unique_lock<boost::mutex> lock(m_mutex);
  debugLines.push_back(Vector3(position + Vector3::UnitX() * size));
  debugLines.push_back(Vector3(position - Vector3::UnitX() * size));
  debugLines.push_back(Vector3(position + Vector3::UnitY() * size));
  debugLines.push_back(Vector3(position - Vector3::UnitY() * size));
  debugLines.push_back(Vector3(position + Vector3::UnitZ() * size));
  debugLines.push_back(Vector3(position - Vector3::UnitZ() * size));
}

void DebugDrawUtil::DrawArrow(const Vector3& from, const Vector3& to) {
  boost::unique_lock<boost::mutex> lock(m_mutex);
  debugLines.push_back(from);
  debugLines.push_back(to);
  // todo: draw arrow head
}

void DebugDrawUtil::FlushDebugPrimitives() {
  boost::unique_lock<boost::mutex> lock(m_mutex);
  const int primNames[] = { GL_POINTS, GL_LINES, GL_TRIANGLES };
  std::vector<Vector3>* primBuffers[] = { &debugPoints, &debugLines, &debugTriangles };
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
    buffer.clear();
  }
  glPolygonMode(GL_FRONT, GL_FILL);
  glEnable(GL_DEPTH_TEST);
}

