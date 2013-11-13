#ifndef __FILES_H__
#define __FILES_H__

#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <set>
#include "Mesh.h"

/**
* Handle files (import/export)
* @author Stéphane GINIER
*/
class Files
{

public:
  Files();
  ~Files();

  Mesh* loadSTL(std::istream& stream) const;
  Mesh* loadPLY(std::istream& stream) const;
  Mesh* loadOBJ(std::istream& stream) const;
  Mesh* load3DS(std::istream& stream) const;
  int detectNewVertex(const Vertex &v, int iTri, VertexSet &setVertices, VertexVector &vertices) const;

  void saveSTL(Mesh* mesh, const std::string& filename) const;
  void saveOBJ(Mesh* mesh, std::ostream& ss) const;
  void savePLY(Mesh* mesh, std::ostream& ss) const;

};

#endif /*__FILES_H__*/
