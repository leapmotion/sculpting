#ifndef __AABB_H__
#define __AABB_H__

#include "DataTypes.h"
#include "Vertex.h"

/**
 * Axis Aligned Bounding Box (AABB)
 * @author Stéphane GINIER
 */
class Aabb
{
public:
    Aabb(const Vector3& min = Vector3::Zero(), const Vector3& max = Vector3::Zero());
    ~Aabb();
    Vector3 getCenter() const;

    bool pointInside(const Vector3& vert) const;
    bool isOutside(const Aabb &aabb) const;
    bool isInside(const Aabb &aabb)const;
    void expand(const Vector3& vert);
    void expand(const Aabb &aabb);
    bool intersectRay(const Vector3& vert, const Vector3& dir) const;
    bool intersectSphere(const Vector3& vert, float radiusSquared) const;
    void checkFlat(float offset);
    void draw(const QColor &color) const;
		Vector3 closestPoint(const Vector3& vert) const;

public :
    Vector3 min_; //min vertex
    Vector3 max_; //max vertex
};

#endif /*__AABB_H__*/
