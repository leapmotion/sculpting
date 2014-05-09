#include "StdAfx.h"
#include "Picking.h"
#include <algorithm>

/** Constructor */
Picking::Picking() : _mesh(0), pickedTriangle_(-1), pickedVertices_(), intersectionPoint_(Vector3::Zero()),
  radiusScreen_(50), radiusWorldSquared_(0)
{}

/** Destructor */
Picking::~Picking()
{}

/** Getters/Setters */
Mesh* Picking::getMesh() { return _mesh; }
void Picking::setMesh(Mesh *mesh) { _mesh=mesh; }
void Picking::setIntersectionPoint(const Vector3& intersectionPoint) { intersectionPoint_ = intersectionPoint; }
const Vector3& Picking::getIntersectionPoint() const { return intersectionPoint_; }
int Picking::getSelectedTriangle() const { return pickedTriangle_; }
std::vector<int>& Picking::getSelectedVertices() { return pickedVertices_; }
void Picking::setRadiusWorldSquared(float radiusWorldSquared) { radiusWorldSquared_ =  radiusWorldSquared; }
float Picking::getRadiusWorldSquared() const { return radiusWorldSquared_; }
void Picking::setRadiusScreen(float radius) { radiusScreen_=radius; }
float Picking::getRadiusScreen() const { return radiusScreen_; }

/** Intersection between a ray and a mesh */
void Picking::intersectionRayMesh(Mesh *mesh, const Vector3& vertexNear, const Vector3& vertexFar,
                                  const Vector2& mouseXY, const Vector2& viewport)
{
  _mesh = 0;
  pickedTriangle_ = -1;
  VertexVector &vertices = mesh->getVertices();
  TriangleVector &triangles = mesh->getTriangles();
  std::vector<int> iTrisCandidates;
  mesh->getOctree()->intersectRay(vertexNear,(vertexFar-vertexNear).normalized(), iTrisCandidates);
  float distance = std::numeric_limits<float>::max();
  int nbTrisCandidates = iTrisCandidates.size();
#pragma omp parallel for
  for (int i = 0; i<nbTrisCandidates; ++i)
  {
    Vector3 vertInter(Vector3::Zero());
    Triangle &t=triangles[iTrisCandidates[i]];
    if (Geometry::intersectionRayTriangle(vertexNear, vertexFar,
      vertices[t.vIndices_[0]],
      vertices[t.vIndices_[1]],
      vertices[t.vIndices_[2]],
      t.normal_, vertInter))
    {
      float testDistance = (vertexNear-vertInter).squaredNorm();
#pragma omp critical
      {
        if (testDistance<distance)
        {
          distance=testDistance;
          intersectionPoint_ = vertInter;
          pickedTriangle_ = iTrisCandidates[i];
        }
      }
    }
  }
  if(pickedTriangle_!=-1)
  {
    _mesh = mesh;
    double z = 0.0;
    Vector4 intersection(Vector4::Zero());
    intersection << intersectionPoint_, 0.0;
    Geometry::point3Dto2D((_mesh->getTransformation()*intersection).head<3>(),viewport,z);
    Vector3 vCircle = Geometry::point2Dto3D(mouseXY+Vector2(radiusScreen_,0.0f),viewport,static_cast<float>(z));
    radiusWorldSquared_ = ((_mesh->getTransformation()*intersection).head<3>()-vCircle).squaredNorm();
  }
  else
    radiusWorldSquared_ = 0;
}

/** Find all the vertices inside the sphere */
void Picking::setPickedVerticesInsideSphere(float radiusWorldSquared)
{
  pickedVertices_.clear();
  VertexVector &vertices = _mesh->getVertices();
  std::vector<Octree*> &leavesHit = _mesh->getLeavesUpdate();
  std::vector<int> iTrisInCells;
  _mesh->getOctree()->intersectSphere(intersectionPoint_,radiusWorldSquared,leavesHit,iTrisInCells);
  std::vector<int> iVerts;
  _mesh->getVerticesFromTriangles(iTrisInCells, iVerts);
  int nbVerts = iVerts.size();
  ++Vertex::sculptMask_;
  for (int i=0;i<nbVerts;++i)
  {
    Vertex &v=vertices[iVerts[i]];
    float distSquared = (v-intersectionPoint_).squaredNorm();
    if(distSquared<radiusWorldSquared)
    {
      v.sculptFlag_ = Vertex::sculptMask_;
      pickedVertices_.push_back(iVerts[i]);
    }
  }
  if(pickedVertices_.empty() && pickedTriangle_!=-1) //no vertices inside the brush radius (big triangle or small radius)
  {
    Triangle &t = _mesh->getTriangle(pickedTriangle_);
    pickedVertices_.push_back(t.vIndices_[0]);
    pickedVertices_.push_back(t.vIndices_[1]);
    pickedVertices_.push_back(t.vIndices_[2]);
  }
}

/** Force the picking even if the the mouse doesn't intersect the mesh */
void Picking::forceCenter(Mesh *mesh, const Vector3& center, const Vector2& mouseXY, const Vector2& viewport)
{
  _mesh = mesh;
  intersectionPoint_ = center;
  double z = 0.0f;
  Vector4 intersection(Vector4::Zero());
  intersection << center, 0.0;
  Geometry::point3Dto2D((_mesh->getTransformation()*intersection).head<3>(),viewport,z);
  Vector3 vCircle = Geometry::point2Dto3D(mouseXY+Vector2(radiusScreen_,0.0f),viewport,static_cast<float>(z));
  radiusWorldSquared_ = ((_mesh->getTransformation()*intersection).head<3>()-vCircle).squaredNorm();
}
