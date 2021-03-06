#ifndef __SCULPT_H__
#define __SCULPT_H__

#include "DataTypes.h"
#include "Mesh.h"
#include "Topology.h"
#include <vector>

class AutoSave;

/**
* Sculpt
* @author St�phane GINIER
*/
class Sculpt
{

public:
  enum SculptMode{INVALID = -1, INFLATE, DEFLATE, SMOOTH, FLATTEN, SWEEP, PUSH, PAINT, CREASE};
  enum TopoMode{DECIMATION, SUBDIVISION, UNIFORMISATION, ADAPTIVE, STATIC};
  Sculpt();
  ~Sculpt();
  void setMesh(Mesh *mesh) { mesh_ = mesh; }
  void toggleCulling();
  void setSculptMode(SculptMode mode) { sculptMode_ = mode; }
  void setTopoMode(TopoMode mode) { topoMode_ = mode; }
  bool isSweep() { return sculptMode_==SWEEP; }
  void setMaterialColor(const Vector3& color) { materialColor_ = color; }
  void setSymmetry(bool symmetry) { symmetry_ = symmetry; }
  bool symmetry() const { return symmetry_; }

  void setRemeshRadius(float remeshRadius) { remeshRadius_ = remeshRadius; }
  void sculptMesh(std::vector<int> &iVertsSelected, const Brush& brush);

  static void setDetail(float detail) { detail_ = detail; }
  static void smooth(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush, bool limit = true);
  static void smoothFlat(Mesh* mesh, const std::vector<int> &iVerts);
  static void draw(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush, bool negate = false);
  static void flatten(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush);
  static void sweep(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush);
  static void push(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush);
  static void crease(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush);
  static void paint(Mesh* mesh, const std::vector<int> &iVerts, const Brush& brush, const Vector3& color);

  int getNumBrushes() const { return (int)_brushes.size(); }
  void addBrush(const Vector3& worldPos, const Vector3& pos, const Vector3& dir, const Vector3& vel, float radius, float strength, float activation);
  void clearBrushes() { _brushes.clear(); }
  void applyBrushes(double curTime, AutoSave* autoSave);
  BrushVector getBrushes() const;
  double getLastSculptTime() const { return lastSculptTime_; }

  std::mutex& getBrushMutex();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  static void setAdaptiveParameters(float radiusSquared);
  static Vector3 areaNormal(Mesh* mesh, const std::vector<int> &iVerts);
  static Vector3 areaCenter(Mesh* mesh, const std::vector<int> &iVerts);
  static void laplacianSmooth(Mesh* mesh, const std::vector<int> &iVerts, Vector3Vector &smoothVerts, Vector3Vector &smoothColors);
  void remesh(float remeshRadius);

private:

  struct MaskMatch {
    MaskMatch(const VertexVector& verts) : vertices(verts) { }
    bool operator()(const int& idx) {
      return vertices[idx].sculptFlag_ != Vertex::sculptMask_;
    }
    const VertexVector& vertices;
  };

  static float detail_; //intensity of details
  static float d2Min_; //uniform refinement of mesh (min edge length)
  static float d2Max_; //uniform refinement of mesh (max edge length)
  static float d2Thickness_; //distance between 2 vertices before split/merge
  static float d2Move_; //max displacement of vertices per step

  Mesh* mesh_; //selected meshs
  SculptMode sculptMode_; //sculpting mode
  TopoMode topoMode_; //topological mode
  bool prevSculpt_;
  int material_;
  Vector3 materialColor_;
  float autoSmoothStrength_;
  std::vector<int> brushVertices_;
  std::vector<int> iTris_;
  mutable std::mutex brushMutex_;
  double lastSculptTime_;
  double lastUpdateTime_;
  Topology topo_;
  float remeshRadius_;
  bool symmetry_;

  BrushVector _brushes;
};

#endif /*__SCULPT_H__*/
