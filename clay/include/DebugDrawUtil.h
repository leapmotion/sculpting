#pragma once
#ifndef __DebugDrawUtil_h__
#define __DebugDrawUtil_h__

#include "DataTypes.h"


class Mesh;
class Triangle;

class lmColor {
public:
  enum ColorName;

  lmColor() { m_color.m_value = 0xffffffff; }
  lmColor(ColorName value) { m_color.m_value = value; }
  lmColor(lmReal red, lmReal green, lmReal blue, lmReal alpha = 1.0f) { m_color.m_red = unsigned __int8(255.0f * red); m_color.m_green = unsigned __int8(255.0f * green); m_color.m_blue = unsigned __int8(255.0f * blue); m_color.m_alpha = unsigned __int8(255.0f * alpha); }

  lmReal red() const { return m_color.m_red / 255.0f; }
  lmReal green() const { return m_color.m_green / 255.0f; }
  lmReal blue() const { return m_color.m_blue / 255.0f; }
  lmReal alpha() const { return m_color.m_alpha / 255.0f; }

  enum ColorName {
    WHITE = 0xffffffff,
    BLACK = 0xff000000,
    RED = 0xffff0000,
    GREEN = 0xff00ff00,
    BLUE = 0xff0000ff,
    YELLOW = 0xffffff00,
    CYAN = 0xff00ffff,
    MAGENTA = 0xffff00ff,
  };

private:
  union {
    struct {
      unsigned __int8 m_blue;
      unsigned __int8 m_green;
      unsigned __int8 m_red;
      unsigned __int8 m_alpha;
    };
    unsigned __int32 m_value;
  } m_color;
};


namespace cinder {
  class GlslProg;
}

class DebugDrawUtil {
public:
  static DebugDrawUtil& getInstance() { static DebugDrawUtil instance; return instance; }

  struct VectorWithColor {
    Vector3 m_vector;
    lmColor m_color;
    VectorWithColor() {}
    VectorWithColor(const Vector3& v, lmColor c) : m_vector(v), m_color(c) {}
  };

  typedef std::vector<VectorWithColor, Eigen::aligned_allocator<VectorWithColor> > VectorWithColorVector;

  struct Buffers {
    VectorWithColorVector m_pointsCol; // debug lines with color
    VectorWithColorVector m_linesCol;
    VectorWithColorVector m_trianglesCol;
    VectorWithColorVector m_facesCol;

    void clearAll();
  };

  template<bool PERM>
  void DrawPoint(const Vector3& a, lmColor color = lmColor::WHITE);

  template<bool PERM>
  void DrawLine(const Vector3& a, const Vector3& b, lmColor color = lmColor::WHITE);

  template<bool PERM>
  void DrawTriangle(const Vector3& a, const Vector3& b, const Vector3& c, lmColor color = lmColor::WHITE);

  template<bool PERM>
  void DrawFace(const Vector3& a, const Vector3& b, const Vector3& c, lmColor color = lmColor::WHITE);

  template<bool PERM>
  void DrawCross(const Vector3& position, lmReal size, lmColor color = lmColor::WHITE);


  void DrawArrow(const Vector3& from, const Vector3& to);

  void DrawTriangle(const Mesh* mesh, const Triangle& tri);

  void DrawFace(const Mesh* mesh, const Triangle& tri);

  void SwitchBuffers();

  inline Buffers& GetDrawingBuffer() { return m_buffers[(1+m_readyBufferIdx)%2]; }
  inline Buffers& GetDrawingPermBuffer() { return m_permBuffers[(1+m_readyBufferIdx)%2]; }

  void FlushDebugPrimitives(cinder::gl::GlslProg* shader);

  Buffers m_buffers[2];
  Buffers m_permBuffers[2];
  int m_readyBufferIdx;

  std::mutex m_mutex;

private:
  DebugDrawUtil() { m_readyBufferIdx = 1; }
  DebugDrawUtil(const DebugDrawUtil& u) { m_readyBufferIdx = 1; }
  ~DebugDrawUtil() {}
};


#define LM_DRAW_POINT(v0, color) DebugDrawUtil::getInstance().DrawPoint<false>(v0, color);
#define LM_DRAW_LINE(v0, v1, color) DebugDrawUtil::getInstance().DrawLine<false>(v0, v1, color);
#define LM_DRAW_TRIANGLE(v0, v1, v2, color) DebugDrawUtil::getInstance().DrawTriangle<false>(v0, v1, v2);
#define LM_DRAW_FACE(v0, v1, v2, color) DebugDrawUtil::getInstance().DrawFace<false>(v0, v1, v2);
#define LM_DRAW_CROSS(v0, size, color) DebugDrawUtil::getInstance().DrawCross<false>(v0, size, color);

#define LM_PERM_POINT(v0, color) DebugDrawUtil::getInstance().DrawPoint<true>(v0, color);
#define LM_PERM_LINE(v0, v1, color) DebugDrawUtil::getInstance().DrawLine<true>(v0, v1, color);
#define LM_PERM_TRIANGLE(v0, v1, v2, color) DebugDrawUtil::getInstance().DrawTriangle<true>(v0, v1, v2, color);
#define LM_PERM_FACE(v0, v1, v2, color) DebugDrawUtil::getInstance().DrawFace<true>(v0, v1, v2, color);
#define LM_PERM_CROSS(v0, size, color) DebugDrawUtil::getInstance().DrawCross<true>(v0, size, color);



#endif // __DebugDrawUtil_h__
