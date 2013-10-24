#ifndef __UNDO_H__
#define __UNDO_H__

#include "Vertex.h"
#include "Triangle.h"

/**
* State
* @author Stéphane GINIER
*/
class State
{
public:
  State();
  ~State();

public:
  int nbTrianglesState_; //number of triangles
  int nbVerticesState_; //number of vertices
  TriangleVector tState_; //copies of some triangles
  VertexVector vState_; //copies of some vertices
  Aabb aabbState_; //root aabb
};

#endif /*__UNDO_H__*/
