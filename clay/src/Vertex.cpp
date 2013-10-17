#include "Vertex.h"

int Vertex::tagMask_ = 1;
int Vertex::sculptMask_ = 1;

/** Constructor */
Vertex::Vertex(float x, float y, float z, int id) : Vector3(x, y, z), tagFlag_(1), sculptFlag_(1),
    stateFlag_(1), id_(id), normal_(Vector3::Zero()), material_(Vector3::Ones())
{}

/** Constructor */
Vertex::Vertex(const Vector3& vec, int id) : Vector3(vec), tagFlag_(1), sculptFlag_(1),
    stateFlag_(1), id_(id), normal_(Vector3::Zero()), material_(Vector3::Ones())
{}

/** Assignment operator */
Vertex& Vertex::operator=(const Vector3& vec)
{
    x() = vec.x();
    y() = vec.y();
    z() = vec.z();
    return *this;
}

/** Destructor */
Vertex::~Vertex()
{}

void Vertex::addTriangle(int iTri) { tIndices_.push_back(iTri); }
void Vertex::addRingVertex(int iVer) { ringVertices_.push_back(iVer); }

/** Replace triangle */
void Vertex::replaceTriangle(int iTriOld, int iTriNew)
{
    int nbTris=tIndices_.size();
    for(int i=0;i<nbTris;++i)
    {
        if(iTriOld==tIndices_[i])
        {
            tIndices_[i]=iTriNew;
            return;
        }
    }
}

/** Replace ring vertex */
void Vertex::replaceRingVertex(int iVerOld, int iVerNew)
{
    int nbVerts=ringVertices_.size();
    for(int i=0;i<nbVerts;++i)
    {
        if(iVerOld==ringVertices_[i])
        {
            ringVertices_[i]=iVerNew;
            return;
        }
    }
}

/** Remove triangle */
void Vertex::removeTriangle(int iTri)
{
    int nbTris=tIndices_.size();
    for(int i=0;i<nbTris;++i)
    {
        if(iTri==tIndices_[i])
        {
            tIndices_[i]=tIndices_.back();
            tIndices_.pop_back();
            return;
        }
    }
}

/** Remove ring vertex */
void Vertex::removeRingVertex(int iVer)
{
    int nbVerts=ringVertices_.size();
    for(int i=0;i<nbVerts;++i)
    {
        if(iVer==ringVertices_[i])
        {
            ringVertices_[i]=ringVertices_.back();
            ringVertices_.pop_back();
            return;
        }
    }
}

/** < operator */
bool operator<( const Vertex  &a, const  Vertex & b)
{
    if (a.x()<b.x())
        return true;
    if (a.x()>b.x())
        return false;
    if (a.y()<b.y())
        return true;
    if (a.y()>b.y())
        return false;
    if (a.z()<b.z())
        return true;
    if (a.z()>b.z())
        return false;
    return false;
}
