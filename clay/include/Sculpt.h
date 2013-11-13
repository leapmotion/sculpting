#ifndef __SCULPT_H__
#define __SCULPT_H__

#include "DataTypes.h"
#include "Mesh.h"
#include "Topology.h"
#include <vector>

/**
* Sculpt
* @author Stéphane GINIER
*/
class Sculpt
{

public:
  enum SculptMode{INVALID = -1, INFLATE, DEFLATE, SMOOTH, FLATTEN, SWEEP, PUSH, PAINT};
  enum TopoMode{DECIMATION, SUBDIVISION, UNIFORMISATION, ADAPTIVE, STATIC};
  Sculpt();
  ~Sculpt();
  void setMesh(Mesh *mesh) { mesh_ = mesh; }
  void toggleCulling();
  void setSculptMode(SculptMode mode) { sculptMode_ = mode; }
  void setTopoMode(TopoMode mode) { topoMode_ = mode; }
  void setDetail(float detail) { detail_ = detail; }
  bool isSweep() { return sculptMode_==SWEEP; }
  void setMaterialColor(const Vector3& color) { materialColor_ = color; }

  void sculptMesh(std::vector<int> &iVertsSelected, const Brush& brush);
  void smooth(const std::vector<int> &iVerts, const Brush& brush);
  void smoothFlat(const std::vector<int> &iVerts);
  void draw(const std::vector<int> &iVerts, const Brush& brush);
  void flatten(const std::vector<int> &iVerts, const Brush& brush);
  void smoothNoMp(const std::vector<int> &iVerts, bool flat = false);
  void sweep(const std::vector<int> &iVerts, const Brush& brush);
  void push(const std::vector<int> &iVerts, const Brush& brush);
  void paint(const std::vector<int> &iVerts, const Brush& brush, const Vector3& color);

  int getNumBrushes() const { return (int)_brushes.size(); }
  void addBrush(const Vector3& worldPos, const Vector3& pos, const Vector3& dir, const Vector3& vel, float radius, float strength, float activation);
  void clearBrushes() { _brushes.clear(); }
  void applyBrushes(double curTime, bool symmetry);
  BrushVector getBrushes() const;
  double getLastSculptTime() const { return lastSculptTime_; }

  std::mutex& getBrushMutex();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void setAdaptiveParameters(float radiusSquared);
  Vector3 areaNormal(const std::vector<int> &iVerts);
  Vector3 areaCenter(const std::vector<int> &iVerts);
  void laplacianSmooth(const std::vector<int> &iVerts, Vector3Vector &smoothVerts, Vector3Vector &smoothColors);

private:

  struct MaskMatch {
    MaskMatch(const VertexVector& vertices) : verticesPtr(&vertices) { }
    bool operator()(const int& idx) {
      const VertexVector& vertices = *verticesPtr;
      return vertices[idx].sculptFlag_ != Vertex::sculptMask_;
    }
    const VertexVector* verticesPtr;
  };

  Mesh* mesh_; //selected meshs
  SculptMode sculptMode_; //sculpting mode
  TopoMode topoMode_; //topological mode
  float detail_; //intensity of details
  float d2Min_; //uniform refinement of mesh (min edge length)
  float d2Max_; //uniform refinement of mesh (max edge length)
  float d2Thickness_; //distance between 2 vertices before split/merge
  float d2Move_; //max displacement of vertices per step
  float minDetailMult_;
  bool prevSculpt_;
  int material_;
  Vector3 materialColor_;
  float autoSmoothStrength_;
  std::vector<int> brushVertices_;
  mutable std::mutex brushMutex_;
  double lastSculptTime_;
  double lastUpdateTime_;

  BrushVector _brushes;
};

#endif /*__SCULPT_H__*/
