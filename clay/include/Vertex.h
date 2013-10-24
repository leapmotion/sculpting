#ifndef __VERTEX_H__
#define __VERTEX_H__

#include "DataTypes.h"
#include <vector>
#include "Tools.h"

/**
* Vertex
* @author Stéphane GINIER
*/
class Vertex : public Vector3
{
public:
  static int tagMask_; //flag mask value (should be always >= tagFlag_)
  static int sculptMask_; //flag mask value (should be always >= sculptFlag_)
  int tagFlag_; //general purpose flag (<0 means the vertex is to be deleted)
  int sculptFlag_; //sculpting flag
  int stateFlag_; //flag for history

public:
  Vertex(float x = 0, float y = 0, float z = 0, int id = -1);
  Vertex(const Vector3& vec, int id=-1);
  Vertex& operator=(const Vector3& vec);
  ~Vertex();
  void addTriangle(int iTri);
  void addRingVertex(int iVer);
  void replaceTriangle(int iTriOld, int iTriNew);
  void replaceRingVertex(int iVerOld, int iVerNew);
  void removeTriangle(int iTri);
  void removeRingVertex(int iVer);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  int id_; //id
  std::vector<int> tIndices_; //triangles indices around the vertex
  Vector3 normal_; //normal
  std::vector<int> ringVertices_; //vertices neighborhood (1-ring)
  Vector3 material_;
};

bool operator<(const Vertex &a,const Vertex &b);

typedef std::vector<Vertex, Eigen::aligned_allocator<Vertex> > VertexVector;

#endif /*__VERTEX_H__*/
