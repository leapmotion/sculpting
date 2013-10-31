#include "StdAfx.h"

#include "DebugDrawUtil.h"
#include "Geometry.h"
#include "Mesh.h"
#include "Triangle.h"

#include <algorithm>
#include <GL/GLU.h>



/** Compute intersection vertex between a ray and a triangle. Returne false if it doesn't intersect. */
bool Geometry::intersectionRayTriangle(const Vector3& s1, const Vector3& s2,
                                       const Vector3& v1, const Vector3& v2,
                                       const Vector3& v3, const Vector3& normal,
                                       Vector3& vertInter)
{
  float dist1 = (s1-v1).dot(normal);
  float dist2 = (s2-v1).dot(normal);
  if ((dist1*dist2)>= 0.0f)
    return false;
  if (dist1==dist2)        //ray parallel to triangle plane
    return false;
  //intersection between ray and triangle
  vertInter = s1+(s2-s1)*(-dist1/(dist2-dist1));
  if (normal.cross(v2-v1).dot(vertInter-v1)<0.0f)
    return false;
  if (normal.cross(v3-v2).dot(vertInter-v2)<0.0f)
    return false;
  if (normal.cross(v1-v3).dot(vertInter-v1)<0.0f)
    return false;
  return true;
}

/** Intersection point between a ray and a plane */
Vector3 Geometry::intersectionRayPlane(const Vector3& s1,const Vector3& s2, const Vector3& v1, const Vector3& normal)
{
  float dist1 = (s1-v1).dot(normal);
  float dist2 = (s2-v1).dot(normal);
  if ((dist1*dist2)>= 0.0f)
    return Vector3::Zero();
  if (dist1==dist2)        //ray parallel to triangle plane
    return Vector3::Zero();
  //intersection between ray and triangle
  return s1+(s2-s1)*(-dist1/(dist2-dist1));
}

/** Projection of mouse coordinate on sphere unit */
Vector3 Geometry::mouseOnUnitSphere(const Vector2& mouseXY)
{
  float tempZ = 1-mouseXY.x()*mouseXY.x()-mouseXY.y()*mouseXY.y();
  float mouseZ= tempZ>0 ? sqrtf(tempZ) : 0;
  Vector3 sourisSphere(mouseXY.x(), mouseXY.y(), mouseZ);
  return sourisSphere.normalized();
}

/** Normalize coordinate mouse between 0 and 1
*/
Vector2 Geometry::normalizedMouse(const Vector2& mouseXY, const Vector2 viewport)
{
  return Vector2(2.0*mouseXY.x()/viewport.x()-1.0,1.0-(2.0*mouseXY.y()/viewport.y()));
}

/** Compute the ray normal to screen (through mouse) */
void Geometry::getRayNormalToScreen(const Vector2& mouseXY, const Vector2& viewport, Vector3& vertexNear, Vector3& vertexFar)
{
  double model[16];
  double projection[16];
  int view[4];
  double dX, dY, dZ;
  glGetIntegerv(GL_VIEWPORT, view);
  glGetDoublev (GL_MODELVIEW_MATRIX, model);
  glGetDoublev (GL_PROJECTION_MATRIX, projection);
  gluUnProject (mouseXY.x(), viewport.y()-mouseXY.y(),0.0, model, projection, view, &dX, &dY, &dZ);
  vertexNear.x() = static_cast<float>(dX);
  vertexNear.y() = static_cast<float>(dY);
  vertexNear.z() = static_cast<float>(dZ);
  gluUnProject (mouseXY.x(), viewport.y()-mouseXY.y(),1.0, model, projection, view, &dX, &dY, &dZ);
  vertexFar.x() = static_cast<float>(dX);
  vertexFar.y() = static_cast<float>(dY);
  vertexFar.z() = static_cast<float>(dZ);
}

/** Compute the unit vector coplanar to screen */
Vector3 Geometry::getCoplanarVectorOnScreen(const Vector2& mouseXYOld, const Vector2& mouseXYNew, const Vector2& viewport)
{
  double model[16];
  double projection[16];
  int view[4];
  double dX1, dX2, dY1, dY2, dZ1, dZ2;
  glGetIntegerv(GL_VIEWPORT, view);
  glGetDoublev (GL_MODELVIEW_MATRIX, model);
  glGetDoublev (GL_PROJECTION_MATRIX, projection);
  gluUnProject (mouseXYOld.x(), viewport.y()-mouseXYOld.y(), 0.0, model, projection, view, &dX1, &dY1, &dZ1);
  gluUnProject (mouseXYNew.x(), viewport.y()-mouseXYNew.y(), 0.0, model, projection, view, &dX2, &dY2, &dZ2);
  return Vector3(static_cast<float>(dX2 - dX1), static_cast<float>(dY2 - dY1), static_cast<float>(dZ2 -dZ1)).normalized();
}

/** Compute the projection of a vertex on a line */
Vector3 Geometry::vertexOnLine(const Vector3& vertex, const Vector3& vertexNear, const Vector3& vertexFar)
{
  Vector3 ab = vertexFar - vertexNear;
  float abSquared = ab.squaredNorm();
  float dot = ab.dot(vertex - vertexNear);
  float t = dot / abSquared;
  return (vertexNear + ab*t);
}

/** Project the mouse coordinate into the world coordinate at a given z */
Vector3 Geometry::point2Dto3D(const Vector2& mouseXY, const Vector2& viewport, float z)
{
  double model[16];
  double projection[16];
  int view[4];
  double dX, dY, dZ;
  glGetIntegerv(GL_VIEWPORT, view);
  glGetDoublev (GL_MODELVIEW_MATRIX, model);
  glGetDoublev (GL_PROJECTION_MATRIX, projection);
  gluUnProject (mouseXY.x(), viewport.y()-mouseXY.y(), z, model, projection, view, &dX, &dY, &dZ);
  return Vector3(static_cast<float>(dX), static_cast<float>(dY), static_cast<float>(dZ));
}

/** Project a vertex onto the screen */
Vector2 Geometry::point3Dto2D(const Vector3& vertex, const Vector2& viewport, double &z)
{
  double model[16];
  double projection[16];
  int view[4];
  double dX, dY;
  glGetIntegerv(GL_VIEWPORT, view);
  glGetDoublev (GL_MODELVIEW_MATRIX, model);
  glGetDoublev (GL_PROJECTION_MATRIX, projection);
  gluProject(vertex.x(), vertex.y(), vertex.z(), model, projection, view, &dX, &dY, &z);
  return Vector2(dX, viewport.y()-dY);
}

/** Return any perpendicular vector to another vector */
Vector3 Geometry::getPerpendicularVector(const Vector3& vec)
{
  Vector3 result(Vector3::Zero());
  if (vec.x() == 0.0f || vec.y() == 0.0f || vec.z() == 0.0f)
  {
    if (vec.x() == 0.0f)
      result.x() = 1.0f;
    else if (vec.y() == 0.0f)
      result.y() = 1.0f;
    else
      result.z() = 1.0f;
  }
  else
  {
    result.x() = vec.z();
    result.y() = vec.z();
    result.z() = -(vec.x()+vec.y());
    result.normalize();
  }
  return result;
}

/** Compute the bounding box of a triangle defined by three vertices */
Aabb Geometry::computeTriangleAabb(const Vector3& v1, const Vector3& v2, const Vector3& v3)
{
  float minX = std::min(std::min(v1.x(),v2.x()),v3.x());
  float minY = std::min(std::min(v1.y(),v2.y()),v3.y());
  float minZ = std::min(std::min(v1.z(),v2.z()),v3.z());

  float maxX = std::max(std::max(v1.x(),v2.x()),v3.x());
  float maxY = std::max(std::max(v1.y(),v2.y()),v3.y());
  float maxZ = std::max(std::max(v1.z(),v2.z()),v3.z());

  return Aabb(Vector3(minX, minY, minZ), Vector3(maxX, maxY, maxZ));
}

/** Compute the inradius (radius of incircle) of a triangle */
float Geometry::computeInradiusSquared(const Vector3& v1, const Vector3& v2, const Vector3& v3)
{
  float a = (v1-v2).norm();
  float b = (v1-v3).norm();
  float c = (v2-v3).norm();
  float halfPerimeter = (a+b+c)*0.5f;
  return (1/halfPerimeter)*(halfPerimeter-a)*(halfPerimeter-b)*(halfPerimeter-c);
}

/** If point is inside the triangle, test the sum of the areas */
bool Geometry::pointInsideTriangle(const Vector3& point, const Vector3& v1, const Vector3& v2, const Vector3& v3)
{
  Vector3 vec1 = v1-v2;
  Vector3 vec2 = v1-v3;
  Vector3 vecP1 = point-v2;
  Vector3 vecP2 = point-v3;
  float total = vec1.cross(vec2).norm();
  float area1 = vec1.cross(vecP1).norm();
  float area2 = vec2.cross(vecP2).norm();
  float area3 = vecP1.cross(vecP2).norm();
  if(fabs(total-(area1+area2+area3))<0.0001f) //magic epsilon...
    return true;
  else
    return false;
}

/** If a sphere intersect a triangle */
bool Geometry::sphereIntersectTriangle(const Vector3& point, float radiusSq,
                                       const Vector3& v1, const Vector3& v2, const Vector3& v3)
{
  if(Geometry::distanceToSegment(point,v1,v2)<radiusSq) return true;
  if(Geometry::distanceToSegment(point,v2,v3)<radiusSq) return true;
  if(Geometry::distanceToSegment(point,v1,v3)<radiusSq) return true;
  return false;
}

/** Minimum distance to a segment */
float Geometry::distanceToSegment(const Vector3& point, const Vector3& v1, const Vector3& v2)
{
  float length = (v2-v1).squaredNorm();
  float t = (point-v1).dot(v2-v1)/length;
  if(t < 0.0) return (point-v1).squaredNorm();
  if(t> 1.0) return (point-v2).squaredNorm();
  Vector3 ptProj = v1 + t*(v2-v1);
  return (point-ptProj).squaredNorm();
}

void Geometry::getClosestPoint(const Geometry::GetClosestPointInput& input, Geometry::GetClosestPointOutput* output) {

  // Get input
  const Vector3& v0 = input.mesh->getVertex(input.tri->vIndices_[0]);
  const Vector3& v1 = input.mesh->getVertex(input.tri->vIndices_[1]);
  const Vector3& v2 = input.mesh->getVertex(input.tri->vIndices_[2]);
  const Vector3& pt = input.point;

  // Projected point
  const Vector3 pr = pt - input.tri->normal_.dot(pt-v0) * input.tri->normal_;

  // Triangle edges
  const Vector3& e0 = v1-v0;
  const Vector3& e1 = v2-v1;
  const Vector3& e2 = v0-v2;

  // Get front direction of the triangle
  const Vector3 n = e0.cross(e1);

  LM_ASSERT(n.dot(input.tri->normal_) >= 0.0f, "Normal on mesh triangle flipped.");

  if (n.squaredNorm() < LM_EPSILON * LM_EPSILON) {
    // Degenerate triangle
    // todo: should handle longest edge
  }

  // Cross point's project with every other edge
  const Vector3 cr0 = e0.cross(pr - v0);
  const Vector3 cr1 = e1.cross(pr - v1);
  const Vector3 cr2 = e2.cross(pr - v2);

  int numNeg = 0;
  int numPos = 0;
  int lastNegIdx = -1;
  int lastPosIdx = -1;

  lmReal dot0 = cr0.dot(n);
  lmReal dot1 = cr1.dot(n);
  lmReal dot2 = cr2.dot(n);

  if (0 <= dot0) { numPos++; lastPosIdx = 0; } else { numNeg++; lastNegIdx = 0; }
  if (0 <= dot1) { numPos++; lastPosIdx = 1; } else { numNeg++; lastNegIdx = 1; }
  if (0 <= dot2) { numPos++; lastPosIdx = 2; } else { numNeg++; lastNegIdx = 2; }

  if (numPos == 3) {
    // inside triangle
    output->position = pr;
    bool positive = 0.0f <= input.tri->normal_.dot(pt);
    output->normal = positive ? input.tri->normal_ : -1.0f * input.tri->normal_;
    output->distance = std::fabs(input.tri->normal_.dot(pt-pr));
  } else if (numPos == 2) {
    // outside triangle, closest to an edge
    lastNegIdx; // index of the edge
    Vector3 edgeStart = input.mesh->getVertex(input.tri->vIndices_[lastNegIdx]); // edge start
    Vector3 edgeEnd = input.mesh->getVertex(input.tri->vIndices_[(lastNegIdx+1)%3]); // edge end
    Vector3 edge = edgeEnd - edgeStart;
    lmReal edgeLength = edge.norm();
    Vector3 edgeDir = edge / edgeLength;
    Vector3 toPoint = pt - edgeStart;
    lmReal dot = edgeDir.dot(toPoint);
    dot = lmClip(dot, 0.0f, edgeLength);
    Vector3 pointProjectedOnSegmet = edgeStart + dot * edgeDir;
    {
      lmReal check = edgeDir.dot(pr - pt);
      check = check;
    }
    output->position = pointProjectedOnSegmet;
    output->normal = (pt - output->position); // if not zero
    output->distance = output->normal.norm();
    if (output->distance < LM_EPSILON) {
      output->normal = input.tri->normal_;
    } else {
      output->normal.normalize();
    }
  } else {
    assert (numPos == 1 && "Error finding closest point");
    output->position = input.mesh->getVertex(input.tri->vIndices_[(lastPosIdx-1+3)%3]); // index of the vertex
    output->normal = (pt - output->position); // if not zero
    output->distance = output->normal.norm();
    if (output->distance < LM_EPSILON) {
      output->normal = input.tri->normal_;
    } else {
      output->normal.normalize();
    }
  }
}
