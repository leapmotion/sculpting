#ifndef __TRIANGLE_H__
#define __TRIANGLE_H__

#include "DataTypes.h"
#include "Aabb.h"
#include <vector>

class Octree;

/**
 * Triangle
 * @author Stéphane GINIER
 */
class Triangle
{
public:
    static int tagMask_; //flag mask value (always >= tagFlag_)
    int tagFlag_; //general purpose flag (<0 means the triangle is to be deleted)
    int stateFlag_; //flag for history
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    Triangle(const Vector3& n = Vector3::Zero(),int iVer1 = -1, int iVer2 = -1, int iVer3 = -1, int id = -1);
    ~Triangle();
    void replaceVertex(int iVerOld, int iVerNew);

public:
    int id_; //id
    int vIndices_[3]; //indices of vertices
    Vector3 normal_; //normal of triangle
    Aabb aabb_; //bounding box of the triangle
    Octree *leaf_; //octree leaf
    int posInLeaf_; //position index in the leaf
};

typedef std::vector<Triangle, Eigen::aligned_allocator<Triangle> > TriangleVector;

#endif /*__TRIANGLE_H__*/
