#ifndef __PICKING_H__
#define __PICKING_H__

#include <limits>
#include "DataTypes.h"
#include "Mesh.h"
#include "Geometry.h"
#include "Topology.h"

/**
* Picking
* @author Stéphane GINIER
*/
class Picking
{

public:
  Picking();
  ~Picking();
  void setMesh(Mesh *mesh);
  Mesh* getMesh();
  void setIntersectionPoint(const Vector3 &intersectionPoint);
  const Vector3& getIntersectionPoint() const;
  int getSelectedTriangle() const;
  std::vector<int>& getSelectedVertices();
  void setRadiusWorldSquared(float radiusWorldSquared);
  float getRadiusWorldSquared() const;
  void setRadiusScreen(float radius);
  float getRadiusScreen() const;

  void intersectionRayMesh(Mesh *mesh,const Vector3 &vertexNear, const Vector3 &vertexFar,
    const Vector2 &mouseXY, const Vector2 &viewport);
  void setPickedVerticesInsideSphere(float radiusWorldSquared);
  void forceCenter(Mesh *mesh, const Vector3 &center, const Vector2 &mouseXY, const Vector2 &viewport);

private:
  Mesh *mesh_; //selected mesh
  int pickedTriangle_; //triangle picked
  std::vector<int> pickedVertices_; //vertices selected
  Vector3 intersectionPoint_; //intersection point
  float radiusScreen_; //radius of the selection area (screen unit)
  float radiusWorldSquared_; //radius of the selection area (world unit)
};

#endif /*__PICKING_H__*/
