#ifndef __MESH_H__
#define __MESH_H__

#include <cinder/gl/gl.h>
#include <cinder/Thread.h>
#include "DataTypes.h"
#include <string>
#include <algorithm>
#include <stdint.h>
#include <list>
#include "Tools.h"
#include "Triangle.h"
#include "Vertex.h"
#include "State.h"
#if _WIN32
#include "omp.h"
#endif
#include "Geometry.h"
#include "GLBuffer.h"
#include "Brush.h"

class Octree;

/**
* Mesh
* @author Stéphane GINIER
*/
class Mesh
{
public:
  static const float globalScale_; //for precision issue...
  static int stateMask_; //for history

public:
  Mesh();
  ~Mesh();
  TriangleVector& getTriangles();
  const TriangleVector& getTriangles() const;
  VertexVector& getVertices();
  const VertexVector& getVertices() const;
  std::vector<Octree*>& getLeavesUpdate();
  Triangle& getTriangle(int i);
  const Triangle& getTriangle(int i) const;
  Vertex& getVertex(int i);
  const Vertex& getVertex(int i) const;
  int getNbTriangles() const;
  int getNbVertices() const;
  Vector3 getCenter() const;
  Octree* getOctree() const;
  float getScale() const;
  void setIsSelected(bool);
  Matrix4x4 getTransformation() const;
  Matrix4x4 getTransformation(double curTime) const;
  const Matrix4x4& getRotationMatrix() const;
  const Vector3& getTranslation() const;
  void setRotationVelocity(float vel);
  void updateRotation(double curTime);
  const Vector3& getRotationOrigin() const;
  const Vector3& getRotationAxis() const;
  float getRotationVelocity() const;

  void getTrianglesFromVertices(const std::vector<int> &iVerts, std::vector<int>& triangles);
  void getVerticesFromTriangles(const std::vector<int> &iTris, std::vector<int>& vertices);
  void expandTriangles(std::vector<int> &iTris, int nRing);
  void expandVertices(std::vector<int> &iVerts, int nRing);
  void computeRingVertices(int iVert);
  void getVerticesInsideSphere(const Vector3& point, float radiusWorldSquared, std::vector<int>& result);
  void getVerticesInsideBrush(const Brush& brush, std::vector<int>& result);

  Vector3 getTriangleCenter(int iTri) const;
  void moveTo(const Vector3& destination);

  void draw(GLint vertex, GLint normal, GLint color);
  void drawVerticesOnly(GLint vertex);
  void drawOctree() const;
  bool initMesh();

  void updateMesh(const std::vector<int> &iTris, const std::vector<int> &iVerts);
  void updateGPUBuffers();

  std::vector<int> subdivide(std::vector<int> &iTris,std::vector<int> &iVerts,float inradiusMaxSquared);
  void triangleSubdivision(int iTri);

  void checkLeavesUpdate();

  //undo-redo
  TriangleVector& getTrianglesState();
  VertexVector& getVerticesState();
  void startPushState();
  void pushState(const std::vector<int> &iTris, const std::vector<int> &iVerts);
  void undo();
  void redo();
  void handleUndoRedo();
  void recomputeOctree(const Aabb &aabbSplit);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  void updateOctree(const std::vector<int> &iTris);
  void computeTriangleNormals(const std::vector<int> &iTris);
  void computeVertexNormals(const std::vector<int> &iVerts);
  float angleTri(int iTri, int iVer);
  void initIndexVBO();
  void initVertexVBO();
  void reinitVerticesBuffer();
  void reinitIndicesBuffer();
  void performUndo();
  void performRedo();

  VertexVector vertices_; //vertices
  TriangleVector triangles_; //triangles
  GLint verticesBufferCount_;
  GLint indicesBufferCount_;
  GLBuffer verticesBuffer_; //vertices buffer (openGL)
  GLBuffer normalsBuffer_; //normals buffer (openGL)
  GLBuffer indicesBuffer_; //indexes (openGL)
  GLBuffer colorsBuffer_;
  bool reallocateVerticesBuffer_;
  bool reallocateIndicesBuffer_;
  int pendingGPUTriangles;
  int nbGPUTriangles;
  int pendingGPUVertices;
  Vector3 center_; //center of mesh
  float scale_; //scale
  Octree *octree_; //octree
  //Matrix4x4 matTransform_; //transformation matrix of the mesh
  Matrix4x4 rotationMatrix_;
  Vector3 translation_;
  std::vector<Octree*> leavesUpdate_; //leaves of the octree to check
  bool undoPending_;
  bool redoPending_;
  double lastUpdateTime_;

  //undo-redo
  std::list<State> undo_; //undo actions
  std::list<State> redo_; //redo actions
  std::list<State>::iterator undoIte_; //iterator to undo
  bool beginIte_; //end of undo action

  Vector3 rotationOrigin_;
  Vector3 rotationAxis_;
  float rotationVelocity_;
  float curRotation_;
  Utilities::ExponentialFilter<float> rotationVelocitySmoother_;

  struct IndexUpdate {
    int idx;
    GLuint indices[3];
  };

  struct VertexUpdate {
    int idx;
    Vector3 pos;
    Vector3 color;
    Vector3 normal;
  };

  typedef std::vector<IndexUpdate, Eigen::aligned_allocator<IndexUpdate> > IndexUpdateVector;
  typedef std::vector<VertexUpdate, Eigen::aligned_allocator<VertexUpdate> > VertexUpdateVector;

  IndexUpdateVector indexUpdates_;
  VertexUpdateVector vertexUpdates_;
  std::mutex bufferMutex_;

  // Adrian's
 public:
  void verifyMesh();
};

#endif /*__MESH_H__*/
