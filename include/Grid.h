#ifndef __GRID_H__
#define __GRID_H__

#include <vector>
#include <algorithm>
#include "Tools.h"
#include "Triangle.h"
#include "Geometry.h"
#include "Aabb.h"
#include "Mesh.h"

/**
* Grid
* @author St�phane GINIER
*/
class Grid
{
public:
  Grid();
  ~Grid();
  void init(const Aabb& aabb, float cellSize);
  void build(Mesh *mesh, const std::vector<int> &iVerts);
  void getNeighborhood(const Vector3& v, std::vector<int>& iNearVerts);
  inline int getIndex(int x, int y, int z) { return (x + y*dimX_ + z*dimXY_); }

private:
  Aabb aabb_; //aabb
  int dimX_; //width
  int dimY_; //length
  int dimZ_; //height
  int dimXY_; //width*height (useless optimisation)
  float cellSize_; //size of cell
  int size_; //total size of array
  std::vector< std::vector<int> > iVerts_; //3D grid as a 1-dimensional array
};

#endif /*__OCTREE_H__*/
