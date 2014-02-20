#include "StdAfx.h"
#include "Aabb.h"
#include <cinder/gl/gl.h>

/** Constructor  */
Aabb::Aabb(const Vector3& min, const Vector3& max) : min_(min), max_(max)
{}

/** Destructor  */
Aabb::~Aabb()
{}

/** Setters/Getters */
Vector3 Aabb::getCenter() const { return (max_+min_)/2; }

lmReal Aabb::getDiagonalLength() const {
  return (max_-min_).norm();
}

/** Collision detection) */
bool Aabb::isOutside(const Aabb &aabb) const
{
  if( aabb.min_.x() > max_.x() || aabb.max_.x() < min_.x() )
    return true;
  if( aabb.min_.y() > max_.y() || aabb.max_.y() < min_.y() )
    return true;
  if( aabb.min_.z() > max_.z() || aabb.max_.z() < min_.z() )
    return true;
  return false;
}

/** Return true if aabb is inside the box */
bool Aabb::isInside(const Aabb &aabb)const
{
  if( min_.x() >= aabb.min_.x() && max_.x() <= aabb.max_.x() &&
    min_.y() >= aabb.min_.y() && max_.y() <= aabb.max_.y() &&
    min_.z() >= aabb.min_.z() && max_.z() <= aabb.max_.z() )
    return true;
  return false;
}

/** Return true if vert is inside the aabb */
bool Aabb::pointInside(const Vector3& vert) const
{
  if(vert.x()<=min_.x())
    return false;
  if(vert.x()>max_.x())
    return false;
  if(vert.y()<=min_.y())
    return false;
  if(vert.y()>max_.y())
    return false;
  if(vert.z()<=min_.z())
    return false;
  if(vert.z()>max_.z())
    return false;
  return true;
}

/** Change the size of the aabb to include vert */
void Aabb::expand(const Vector3& vert)
{
  min_ = min_.cwiseMin(vert);
  max_ = max_.cwiseMax(vert);
}

/** Change the size of the aabb to include another aabb */
void Aabb::expand(const Aabb &aabb)
{
  min_ = min_.cwiseMin(aabb.min_);
  max_ = max_.cwiseMax(aabb.max_);
}

/** Return true if a ray intersection the box */
bool Aabb::intersectRay(const Vector3& vert, const Vector3& dir) const
{
  float t1 = (min_.x() - vert.x())/dir.x();
  float t2 = (max_.x() - vert.x())/dir.x();
  float t3 = (min_.y() - vert.y())/dir.y();
  float t4 = (max_.y() - vert.y())/dir.y();
  float t5 = (min_.z() - vert.z())/dir.z();
  float t6 = (max_.z() - vert.z())/dir.z();

  float tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
  float tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));
  return (tmax >=0 && tmin<tmax);
}

/** Check if the aabb is a plane */
void Aabb::checkFlat(float offset)
{
  if(min_.x()==max_.x())
  {
    min_.x() = min_.x()-offset;
    max_.x() = max_.x()+offset;
  }
  if(min_.y()==max_.y())
  {
    min_.y() = min_.y()-offset;
    max_.y() = max_.y()+offset;
  }
  if(min_.z()==max_.z())
  {
    min_.z() = min_.z()-offset;
    max_.z() = max_.z()+offset;
  }
}

/** Draw the aabb */
void Aabb::draw() const
{
  glBegin(GL_LINES);
  glVertex3f(min_.x(), min_.y(), min_.z());glVertex3f(max_.x(), min_.y(), min_.z());
  glVertex3f(max_.x(), min_.y(), min_.z());glVertex3f(max_.x(), max_.y(), min_.z());
  glVertex3f(max_.x(), max_.y(), min_.z());glVertex3f(min_.x(), max_.y(), min_.z());
  glVertex3f(min_.x(), max_.y(), min_.z());glVertex3f(min_.x(), min_.y(), min_.z());
  glVertex3f(min_.x(), min_.y(), min_.z());glVertex3f(min_.x(), min_.y(), max_.z());
  glVertex3f(min_.x(), min_.y(), max_.z());glVertex3f(max_.x(), min_.y(), max_.z());
  glVertex3f(max_.x(), min_.y(), max_.z());glVertex3f(max_.x(), min_.y(), min_.z());
  glVertex3f(max_.x(), min_.y(), max_.z());glVertex3f(max_.x(), max_.y(), max_.z());
  glVertex3f(max_.x(), max_.y(), max_.z());glVertex3f(max_.x(), max_.y(), min_.z());
  glVertex3f(max_.x(), max_.y(), max_.z());glVertex3f(min_.x(), max_.y(), max_.z());
  glVertex3f(min_.x(), max_.y(), max_.z());glVertex3f(min_.x(), max_.y(), min_.z());
  glVertex3f(min_.x(), max_.y(), max_.z());glVertex3f(min_.x(), min_.y(), max_.z());
  glEnd();
}
