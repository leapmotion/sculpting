#include "StdAfx.h"
#include "DebugDrawUtil.h"
#include "Mesh.h"
#include "Triangle.h"

#include "cinder/gl/GlslProg.h"



void DebugDrawUtil::SwitchBuffers() {
  std::unique_lock<std::mutex> lock(m_mutex);
  Buffers& permSrc = GetDrawingPermBuffer();
  m_readyBufferIdx = 1 - m_readyBufferIdx;

  Buffers& b = GetDrawingBuffer();
  b.clearAll();

  Buffers& permDst = GetDrawingPermBuffer();
  permDst = permSrc;
}

void DebugDrawUtil::FlushDebugPrimitives( cinder::gl::GlslProg* shader ) {
  std::unique_lock<std::mutex> lock(m_mutex);

  Buffers* buffers[] = { &m_buffers[m_readyBufferIdx], &m_permBuffers[m_readyBufferIdx] };
  for (int bi = 0; bi < 2; bi++)
  {
    Buffers& b = *buffers[bi];

    const int primNames[] = { GL_TRIANGLES, GL_TRIANGLES, GL_LINES, GL_POINTS };
    VectorWithColorVector* primBuffers[] = { &b.m_facesCol, &b.m_trianglesCol, &b.m_linesCol, &b.m_pointsCol };
    glDisable(GL_DEPTH_TEST);
    glPointSize(2.0f);
    glLineWidth(2.0f);
    glPolygonMode(GL_FRONT, GL_LINE);
    for (int pi = 0; pi < 4; pi++) {
      // Display debug buffers
      VectorWithColorVector& buffer = *primBuffers[pi];
      for (size_t i = 0; i < buffer.size(); i++) {
        const Vector3& v = buffer[i].m_vector;
        const lmColor c = buffer[i].m_color;

        shader->uniform( "surfaceColor",cinder::Color(c.red(), c.green(), c.blue()));
        glBegin(primNames[pi]);
        switch(primNames[pi]) {
        case GL_TRIANGLES:
          {
            const Vector3& w = buffer[i+1].m_vector;
            const Vector3& x = buffer[i+2].m_vector;
            glVertex3f(v.x(), v.y(), v.z());
            glVertex3f(w.x(), w.y(), w.z());
            glVertex3f(x.x(), x.y(), x.z());
            i+=2;
          }
          break;
        case GL_LINES:
          {
            const Vector3& w = buffer[i+1].m_vector;
            glVertex3f(v.x(), v.y(), v.z());
            glVertex3f(w.x(), w.y(), w.z());
            i++;
          }
          break;
        case GL_POINTS:
          glVertex3f(v.x(), v.y(), v.z());
          break;
        }
        glEnd();
      }
    }
    glPolygonMode(GL_FRONT, GL_FILL);
    glEnable(GL_DEPTH_TEST);
  }
}

static inline DebugDrawUtil::VectorWithColor VertexAndColor(const Vector3& v, lmColor color) { return DebugDrawUtil::VectorWithColor(v, color); }

template <bool PERM>
void DebugDrawUtil::DrawPoint( const Vector3& a, lmColor color /*= lmColor::WHITE*/)
{
  DebugDrawUtil::VectorWithColorVector& buffer = PERM ? GetDrawingPermBuffer().m_pointsCol : GetDrawingBuffer().m_pointsCol;
  buffer.push_back(VertexAndColor(a, color));
}

template <bool PERM>
void DebugDrawUtil::DrawLine( const Vector3& a, const Vector3& b, lmColor color /*= lmColor::WHITE*/)
{
  DebugDrawUtil::VectorWithColorVector& buffer = PERM ? GetDrawingPermBuffer().m_linesCol : GetDrawingBuffer().m_linesCol;
  buffer.push_back(VertexAndColor(a, color));
  buffer.push_back(VertexAndColor(b, color));
}

template <bool PERM>
void DebugDrawUtil::DrawTriangle( const Vector3& a, const Vector3& b, const Vector3& c, lmColor color /*= lmColor::WHITE*/)
{
  DebugDrawUtil::VectorWithColorVector& buffer = PERM ? GetDrawingPermBuffer().m_trianglesCol : GetDrawingBuffer().m_trianglesCol;
  buffer.push_back(VertexAndColor(a, color));
  buffer.push_back(VertexAndColor(b, color));
  buffer.push_back(VertexAndColor(c, color));
}

void DebugDrawUtil::DrawMeshTriangle(const Mesh* mesh, const Triangle& tri, lmColor color) {
  LM_DRAW_TRIANGLE(
    mesh->getVertex(tri.vIndices_[0]),
    mesh->getVertex(tri.vIndices_[1]),
    mesh->getVertex(tri.vIndices_[2]),
    color);
}

template <bool PERM>
void DebugDrawUtil::DrawFace( const Vector3& a, const Vector3& b, const Vector3& c, lmColor color /*= lmColor::WHITE*/)
{
  DebugDrawUtil::VectorWithColorVector& buffer = PERM ? GetDrawingPermBuffer().m_facesCol : GetDrawingBuffer().m_facesCol;
  buffer.push_back(VertexAndColor(a, color));
  buffer.push_back(VertexAndColor(b, color));
  buffer.push_back(VertexAndColor(c, color));
}

template<bool PERM>
void DebugDrawUtil::DrawCross( const Vector3& position, lmReal size, lmColor color /*= lmColor::WHITE*/ )
{
  DebugDrawUtil::VectorWithColorVector& buffer = PERM ? GetDrawingPermBuffer().m_linesCol : GetDrawingBuffer().m_linesCol;

  buffer.push_back(VertexAndColor(Vector3(position + Vector3::UnitX() * size), color));
  buffer.push_back(VertexAndColor(Vector3(position - Vector3::UnitX() * size), color));
  buffer.push_back(VertexAndColor(Vector3(position + Vector3::UnitY() * size), color));
  buffer.push_back(VertexAndColor(Vector3(position - Vector3::UnitY() * size), color));
  buffer.push_back(VertexAndColor(Vector3(position + Vector3::UnitZ() * size), color));
  buffer.push_back(VertexAndColor(Vector3(position - Vector3::UnitZ() * size), color));
}

template<bool PERM>
void DebugDrawUtil::DrawArrow(const Vector3& from, const Vector3& to, lmColor color /*= lmColor::WHITE*/ ) {
  if (PERM) {
    LM_PERM_LINE(from, to, color);
  } else {
    LM_DRAW_LINE(from, to, color);
  }

  // Draw arrow head
  lmQuat r; r.setFromTwoVectors(Vector3::UnitZ(), (to-from).normalized());

  Vector3 lines[] = {
    -Vector3::UnitZ() + Vector3::UnitX(),
    -Vector3::UnitZ() - Vector3::UnitX(),
    -Vector3::UnitZ() + Vector3::UnitY(),
    -Vector3::UnitZ() - Vector3::UnitY()
  };

  const lmReal scale = (to-from).norm() * 0.1f;
  for (int i = 0; i < 4; i++)
  {
    Vector3 line = r * lines[i] * scale;
    if (PERM ) {
      LM_PERM_LINE(to, to+line, color);
    } else {
      LM_DRAW_LINE(to, to+line, color);
    }
  }
}



// explicity instantiation without <>
template void DebugDrawUtil::DrawPoint<true>(const Vector3& a, lmColor color);
template void DebugDrawUtil::DrawPoint<false>(const Vector3& a, lmColor color);
template void DebugDrawUtil::DrawLine<true>(const Vector3& a, const Vector3& b, lmColor color);
template void DebugDrawUtil::DrawLine<false>(const Vector3& a, const Vector3& b, lmColor color);
template void DebugDrawUtil::DrawTriangle<true>( const Vector3& a, const Vector3& b, const Vector3& c, lmColor color /*= lmColor::WHITE*/);
template void DebugDrawUtil::DrawTriangle<false>( const Vector3& a, const Vector3& b, const Vector3& c, lmColor color /*= lmColor::WHITE*/);
template void DebugDrawUtil::DrawFace<true>( const Vector3& a, const Vector3& b, const Vector3& c, lmColor color /*= lmColor::WHITE*/);
template void DebugDrawUtil::DrawFace<false>( const Vector3& a, const Vector3& b, const Vector3& c, lmColor color /*= lmColor::WHITE*/);
template void DebugDrawUtil::DrawCross<true>( const Vector3& position, lmReal size, lmColor color /*= lmColor::WHITE*/ );
template void DebugDrawUtil::DrawCross<false>( const Vector3& position, lmReal size, lmColor color /*= lmColor::WHITE*/ );
template void DebugDrawUtil::DrawArrow<true>(const Vector3& a, const Vector3& b, lmColor color);
template void DebugDrawUtil::DrawArrow<false>(const Vector3& a, const Vector3& b, lmColor color);



void DebugDrawUtil::Buffers::clearAll()
{
  m_pointsCol.clear();
  m_linesCol.clear();
  m_trianglesCol.clear();
  m_facesCol.clear();
}
