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
 * @author Stéphane GINIER
 */
class Grid
{
public:
    Grid(const Aabb &aabb);
    ~Grid();
    void init(float cellSize);
    void build(Mesh *mesh, const std::vector<int> &iVerts);
    std::vector<int> getNeighborhood(const Vector3& v);
    inline int getIndex(int x, int y, int z) { return (x + y*dimX_ + z*dimXY_); }

private:
    Aabb aabb_; //aabb
    int dimX_; //width
    int dimY_; //length
    int dimZ_; //height
    int dimXY_; //width*height (useless optimisation)
    float cellSize_; //size of cell
    int size_; //total size of array
    std::vector<int> *iVerts_; //3D grid as a 1-dimensional array
};

#endif /*__OCTREE_H__*/
