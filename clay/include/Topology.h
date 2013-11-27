#ifndef __TOPOLOGY_H__
#define __TOPOLOGY_H__

#include <cinder/gl/gl.h>
#include <limits>
#include <vector>
#include <map>
#include <algorithm>
#include "DataTypes.h"
#include "Mesh.h"
#include "Grid.h"
#include "Octree.h"

/**
* Topology functions
* @author Stéphane GINIER
*/
class Topology
{
private:
  class Edge
  {
  public :
    Edge(int v1 = -1, int v2 = -1, int t = -1) : v1_(v1), v2_(v2), t_(t){}
    ~Edge() {}

  public :
    int v1_;
    int v2_;
    int t_;
  };

public :
  Topology() : mesh_(0), triangles_(0), vertices_(0), centerPoint_(Vector3::Zero()), radiusSquared_(0.0f) {}
  ~Topology() {}
  void init(Mesh *mesh, float radiusSquared, const Vector3& centerPoint) {
    mesh_ = mesh;
    triangles_ = &mesh->getTriangles();
    vertices_ = &mesh->getVertices();
    centerPoint_ = centerPoint;
    radiusSquared_ = radiusSquared;
    verticesMap_.clear();
    iVertsDecimated_.clear();
    iTrisToDelete_.clear();
    iVertsToDelete_.clear();
    iTrisSubd_.clear();
    iVertsSubd_.clear();
    split_.clear();
  }
  void subdivision(std::vector<int> &iTris, float detailMaxSquared);
  void decimation(std::vector<int> &iTris, float detailMinSquared);
  void uniformisation(std::vector<int> &iTris, float detailMinSquared, float detailMaxSquared);
  void adaptTopology(std::vector<int> &iTris, float d2Thickness);

private :
  //subdivision stuffs
  void subdivide(std::vector<int> &iTris, float detail);
  void initSplit(std::vector<int> &iTris, std::vector<int> &iTrisSubd, std::vector<int> &split, float detailSquared);
  void subdivideTriangles(std::vector<int> &iTrisSubd, std::vector<int> &split, float detailSquared);
  void halfEdgeSplit(int iTri, int iv1, int iv2, int iv3);
  void fillTriangles(std::vector<int> &iTris);
  void fillTriangle(int iTri, int iv1, int iv2, int iv3, int ivMid);
  int findSplit(int iTri, float detailSquared, bool checkInsideSphere = false);

  //decimation stuffs
  void decimateTriangles(int iTri1, int iTri2, std::vector<int> &iTris);
  void applyDeletion();
  void getValidModifiedVertices(std::vector<int>& iVerts);
  int findOppositeTriangle(int iTri, int iv1, int iv2);
  void edgeCollapse(int iTri1, int iTri2,int iv1, int iv2,int ivOpp1, int ivOpp2, std::vector<int> &iTris);
  void deleteTriangle(int iTri);
  void deleteVertex(int iVert);

  //adaptive topology stuffs
  void checkCollisions(std::vector<int> &iVerts, float d2Thickness);
  void vertexJoin(int iv1, int iv2);
  void connect1Ring(std::vector<Edge> &edges1, std::vector<Edge> &edges2);
  void connect1RingCommonVertices(std::vector<Edge> &edges1, std::vector<Edge> &edges2, std::vector<int> &common);
  void trianglesRotate(std::vector<int> &iTris, int iv, std::vector<Edge> &edges);
  bool adjustEdgeOrientation(std::vector<Edge> &edges);
  void matchEdgesNearest(std::vector<Edge> &edges1, std::vector<Edge> &edges2);
  void matchEdgesCommonVertices(std::vector<Edge> &edges1, std::vector<Edge> &edges2, int ivCommon);
  void cleanUpNeighborhood(const std::vector<int> &ring1, const std::vector<int> &ring2);
  void cleanUpSingularVertex(int iv);
  bool deleteVertexIfDegenerate(int iv);
  void connectLoop(std::vector<Edge> &edges);
  void connectLinkedEdges(std::vector<Edge> &edges1, std::vector<Edge> &edges2);

private:
  inline TriangleVector& triangles() { return *triangles_; }
  inline VertexVector& vertices() { return *vertices_; }

  Mesh *mesh_; //mesh
  TriangleVector* triangles_; //reference to mesh triangles
  VertexVector* vertices_; //reference to mesh vertices
  Vector3 centerPoint_; //center of brush
  std::map<std::pair<int,int>,int> verticesMap_; //to detect new vertices at the middle of edge (for subdivision)
  float radiusSquared_; //radius squared
  std::vector<int> iVertsDecimated_; //vertices to be updated (mainly for the VBO's, used in decimation and adaptive topo)
  std::vector<int> iTrisToDelete_; //triangles to be deleted
  std::vector<int> iVertsToDelete_; //vertices to be deleted
  std::vector<int> iTrisSubd_;
  std::vector<int> iVertsSubd_;
  std::vector<int> split_;
  Grid grid_;
};

#endif /*__TOPOLOGY_H__*/
