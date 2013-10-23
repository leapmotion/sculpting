#ifndef __OCTREE_H__
#define __OCTREE_H__

#include <vector>
#include <set>
#include <algorithm>
#include "Triangle.h"
#include "Mesh.h"

/**
 * Octree
 * @author Stéphane GINIER
 */
class Octree
{
public:
    static const int maxDepth_ = 8+4;
    static const int maxTriangles_ = 2000 / 10;

public:
    Octree(Octree *parent=0, int depth = 0);
    ~Octree();
    void build(Mesh *mesh, const std::vector<int> &iTris, const Aabb &aabb);
    void constructCells(Mesh *mesh);
    std::vector<int>& getTriangles();
    Aabb& getAabbLoose();
    Aabb& getAabbSplit();
    int getDepth() const;
    Octree* getParent();
    Octree **getChildren();
    void draw() const;
    std::vector<int> intersectRay(const Vector3& vert, const Vector3& dir) const;
    std::vector<int> intersectSphere(const Vector3& vert, float radiusSquared, std::vector<Octree*> &leavesHit);
    void addTriangle(Mesh *mesh, Triangle &tri);
    static void checkEmptiness(Octree* leaf, std::vector<Octree*> &cutLeaves);

private:
    Octree *parent_; //parent
    Octree *child_[8]; //children
    Aabb aabbLoose_; //loose aabb (extended boundary for intersect test)
    Aabb aabbSplit_; //split aabb (static boundary in order to store exactly the triangle according to their center)
    std::vector<int> iTris_; //triangles (if cell is a leaf)
    int depth_; //depth of current node
};

#endif /*__OCTREE_H__*/
