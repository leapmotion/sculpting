#include "Triangle.h"
#include <stdio.h>
#include <stdlib.h>

int Triangle::tagMask_ = 1;

/** Constructor */
Triangle::Triangle(const Vector3& n, int iVer1, int iVer2, int iVer3, int id) : tagFlag_(1), stateFlag_(1),
    id_(id), normal_(n), aabb_(), leaf_(0), posInLeaf_(-1)
{
    vIndices_[0] = iVer1;
    vIndices_[1] = iVer2;
    vIndices_[2] = iVer3;
}

/** Destructor */
Triangle::~Triangle()
{}

/** Replace vertex */
void Triangle::replaceVertex(int iVerOld, int iVerNew)
{
    if(vIndices_[0]==iVerOld) {
        vIndices_[0]=iVerNew;
    } else if(vIndices_[1]==iVerOld) {
        vIndices_[1]=iVerNew;
    } else {
        vIndices_[2]=iVerNew;
    }
}
