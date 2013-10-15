#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include "DataTypes.h"
#include <cinder/gl/gl.h>
#include <Aabb.h>

class Mesh;

/**
 * Geometry functions
 * @author Stéphane GINIER
 */
namespace Geometry
{
    bool intersectionRayTriangle(const Vector3& s1, const Vector3& s2, const Vector3& v1,
                      const Vector3& v2, const Vector3& v3, const Vector3& normal,
                      Vector3& vertInter);
    Vector3 intersectionRayPlane(const Vector3& s1, const Vector3& s2, const Vector3& v1, const Vector3& normal);
    Vector3 mouseOnUnitSphere(const Vector2& mouseXY);
    Vector2 normalizedMouse(const Vector2& mouseXY, const Vector2 window);
    void getRayNormalToScreen(const Vector2& mouseXY, const Vector2& window,
                                  Vector3& vertexNear, Vector3& vertexFar);
    Vector3 getCoplanarVectorOnScreen(const Vector2& mouseXYOld, const Vector2& mouseXYNew, const Vector2& viewport);
    Vector3 vertexOnLine(const Vector3& vertex, const Vector3& vertexNear, const Vector3& vertexFar);
    Vector3 point2Dto3D(const Vector2& mouseXY,const Vector2& viewport, float z=0.0f);
    Vector2 point3Dto2D(const Vector3& vertex,const Vector2& viewport, double &z);
    Vector3 getPerpendicularVector(const Vector3& vec);
    Aabb computeTriangleAabb(const Vector3& v1, const Vector3& v2, const Vector3& v3);
    float computeInradiusSquared(const Vector3& v1, const Vector3& v2, const Vector3& v3);
    bool pointInsideTriangle(const Vector3& point, const Vector3& v1, const Vector3& v2, const Vector3& v3);
    bool sphereIntersectTriangle(const Vector3& point, float radiusSq, const Vector3& v1, const Vector3& v2, const Vector3& v3);
    float distanceToSegment(const Vector3& point, const Vector3& v1, const Vector3& v2);
}

#endif /*__GEOMETRY_H__*/
