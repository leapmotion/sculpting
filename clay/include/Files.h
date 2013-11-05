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
  Mesh* loadSTL(const std::string &filepath) const;
  Mesh* loadPLY(const std::string& filepath) const;
  int detectNewVertex(const Vertex &v, int iTri, VertexSet &setVertices, VertexVector &vertices) const;
  void saveSTL(Mesh* mesh, const std::string& filename) const;
  void saveOBJ(Mesh* mesh, std::ostream& ss) const;
  void savePLY(Mesh* mesh, std::ostream& ss) const;

  Mesh* loadOBJ(const std::string &filepath) const;

  Mesh* load3DS(const std::string &filepath) const;
};

#endif /*__FILES_H__*/
