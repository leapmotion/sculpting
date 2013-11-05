#include "StdAfx.h"
#include "Files.h"
#include <stdlib.h>
#include <sstream>
#include <cstdlib>

/** Constructor */
Files::Files()
{}

/** Destructor */
Files::~Files()
{}

/** Load STL file */
Mesh* Files::loadSTL(const std::string &filepath) const
{
  std::ifstream file(filepath.c_str(), std::ios::binary);
  if (!file) {
    return 0;
  }
  Mesh *mesh = new Mesh();
  VertexSet verticesSet; //detect already read vertices ...
  char header[80];
  int nbTrianglesFile;
  file.read(header, 80); //header
  file.read((char*)&nbTrianglesFile, 4); //number of triangles
  TriangleVector &triangles = mesh->getTriangles();
  VertexVector &vertices = mesh->getVertices();
  int nbTriangles = nbTrianglesFile;
  triangles.reserve(nbTriangles);
  vertices.reserve(nbTriangles/2);
  int iVer1, iVer2, iVer3;
  float x, y, z;
  Vertex v1, v2, v3;
  for (int i=0;i<nbTriangles;++i)
  {
    file.read((char*)&x, 12); //normal
    file.read((char*)&x, 4); //vertex 1
    file.read((char*)&y, 4);
    file.read((char*)&z, 4);
    v1 = Vertex(x, y, z, verticesSet.size());
    iVer1 = detectNewVertex(v1, i, verticesSet,vertices);
    file.read((char*)&x, 4); //vertex 2
    file.read((char*)&y, 4);
    file.read((char*)&z, 4);
    v2 = Vertex(x, y, z, verticesSet.size());
    iVer2 = detectNewVertex(v2, i, verticesSet,vertices);
    file.read((char*)&x, 4); //vertex 3
    file.read((char*)&y, 4);
    file.read((char*)&z, 4);
    v3 = Vertex(x, y, z, verticesSet.size());
    iVer3 = detectNewVertex(v3, i, verticesSet,vertices);
    file.read((char*)&x, 2); //attribute
    triangles.push_back(Triangle((v2-v1).cross(v3-v1).normalized(), iVer1, iVer2, iVer3, i));
  }
  file.close();
  mesh->initMesh();
  mesh->moveTo(Vector3::Zero());
  return mesh;
}

Mesh* Files::loadPLY(const std::string& filepath) const {
  std::ifstream file(filepath.c_str(), std::ios::binary);
  if (!file) {
    return 0;
  }
  Mesh *mesh = new Mesh();
  TriangleVector &triangles = mesh->getTriangles();
  VertexVector &vertices = mesh->getVertices();
  std::string line;
  static const std::string ELEMENT_VERTEX = "element vertex ";
  static const std::string ELEMENT_FACE = "element face ";
  static const std::string END_HEADER = "end_header";
  static const std::string PROPERTY = "property ";
  static const std::string RED = "red";
  int nbVertices = -1;
  int nbFaces = -1;
  int colorIndex = -1;
  int i = 0;
  while (std::getline(file, line)) {
    if (line.find(ELEMENT_VERTEX) == 0) {
      std::stringstream ss;
      std::string remainder = line.substr(ELEMENT_VERTEX.size());
      ss.str(remainder);
      ss >> nbVertices;
      int startIndex = i;
      while (std::getline(file, line)) {
        i++;
        if (line.find(PROPERTY) == 0) {
          if (line.find(RED) != std::string::npos) {
            colorIndex = i - startIndex - 1;
            break;
          }
        } else {
          break;
        }
      }
      i--;
    }
    if (line.find(ELEMENT_FACE) == 0) {
      std::stringstream ss;
      std::string remainder = line.substr(ELEMENT_FACE.size());
      ss.str(remainder);
      ss >> nbFaces;
    }
    if (line.find(END_HEADER) == 0) {
      vertices.reserve(nbVertices);
      triangles.reserve(nbFaces);
      float x, y, z;
      int nbVert, iv1, iv2, iv3, iv4;
      for (int j=0; j<nbVertices; j++) {
        if (!std::getline(file, line)) {
          break;
        }
        std::stringstream ss;
        ss.str(line);
        ss >> x >> y >> z;
        vertices.push_back(Vertex(x, y, z, vertices.size()));
        if (colorIndex >= 0) {
          int curIdx = 3;
          while (curIdx < colorIndex) {
            ss >> x; // remove dummy entries
            curIdx++;
          }
          ss >> x >> y >> z;
          vertices.back().material_ << x/255.0f, y/255.0f, z/255.0f;
        }
      }
      for (int j=0; j<nbFaces; j++) {
        if (!std::getline(file, line)) {
          break;
        }
        std::stringstream ss;
        ss.str(line);
        ss >> nbVert;
        if (nbVert == 3 || nbVert == 4) {
          ss >> iv1 >> iv2 >> iv3;
          Vertex& v1 = vertices[iv1];
          Vertex& v2 = vertices[iv2];
          Vertex& v3 = vertices[iv3];
          int nbTriangles = triangles.size();
          v1.tIndices_.push_back(nbTriangles);
          v2.tIndices_.push_back(nbTriangles);
          v3.tIndices_.push_back(nbTriangles);
          triangles.push_back(Triangle((v2-v1).cross(v3-v1).normalized(), iv1, iv2, iv3, nbTriangles));
          if (nbVert == 4) {
            nbTriangles++;
            ss >> iv4;
            Vertex& v4 = vertices[iv4];
            v1.tIndices_.push_back(nbTriangles);
            v3.tIndices_.push_back(nbTriangles);
            v4.tIndices_.push_back(nbTriangles);
            triangles.push_back(Triangle((v3-v1).cross(v4-v1).normalized(), iv1, iv3, iv4, nbTriangles));
          }
        }
      }
    }
    i++;
  }

  file.close();
  mesh->initMesh();
  mesh->moveTo(Vector3::Zero());
  return mesh;
}

/** Check if the vertex already exists */
int Files::detectNewVertex(const Vertex &v, int iTri, VertexSet &verticesSet, VertexVector &vertices) const
{
  std::pair<VertexSet::iterator, bool> pair = verticesSet.insert(v);
  int iVert = (*pair.first).id_;
  if (pair.second)
    vertices.push_back(v);
  vertices[iVert].addTriangle(iTri);
  return iVert;
}

/** Save file in STL format */
void Files::saveSTL(Mesh* mesh, const std::string& filename) const
{
  std::ofstream file(filename.c_str(), std::ios::binary);
  if (file)
  {
    TriangleVector &triangles = mesh->getTriangles();
    VertexVector &vertices = mesh->getVertices();
    float scale = 1/mesh->getScale();
    char header[80];
    file.write(header, 80);
    int nbTriangles = mesh->getNbTriangles();
    file.write((char*)&nbTriangles, 4);
    for (int i=0;i<nbTriangles;++i)
    {
      Triangle &tri = triangles[i];
      Vector4 normal;
      normal << tri.normal_, 0.0;
      normal = mesh->getTransformation()*normal;
      float xTemp = normal.x();
      float yTemp = normal.y();
      float zTemp = normal.z();
      file.write((char*)&xTemp, 4); //normal
      file.write((char*)&yTemp, 4);
      file.write((char*)&zTemp, 4);
      Vertex v1 = vertices[tri.vIndices_[0]];
      v1*=scale;
      xTemp = v1.x();
      yTemp = v1.y();
      zTemp = v1.z();
      file.write((char*)&xTemp, 4); //vertex 1
      file.write((char*)&yTemp, 4);
      file.write((char*)&zTemp, 4);
      Vertex v2 = vertices[tri.vIndices_[1]];
      v2*=scale;
      xTemp = v2.x();
      yTemp = v2.y();
      zTemp = v2.z();
      file.write((char*)&xTemp, 4); //vertex 2
      file.write((char*)&yTemp, 4);
      file.write((char*)&zTemp, 4);
      Vertex v3 = vertices[tri.vIndices_[2]];
      v3*=scale;
      xTemp = v3.x();
      yTemp = v3.y();
      zTemp = v3.z();
      file.write((char*)&xTemp, 4); //vertex 3
      file.write((char*)&yTemp, 4);
      file.write((char*)&zTemp, 4);
      char attribute[2];
      file.write((char*)attribute, 2); //attribut
    }
    file.close();
  }
}

void Files::saveOBJ(Mesh* mesh, std::ostream& ss) const {
  if (!ss) {
    return;
  }
  const TriangleVector &triangles = mesh->getTriangles();
  const VertexVector &vertices = mesh->getVertices();
  const float scale = 1/mesh->getScale();
  const int nbTriangles = triangles.size();
  const int nbVertices = vertices.size();
  ss << "s 0" << std::endl;
  for (int i=0; i<nbVertices; i++) {
    const Vector3 cur = scale*vertices[i];
    ss << "v " << cur.x() << " " << cur.y() << " " << cur.z() << std::endl;
  }
  for (int i=0; i<nbTriangles; i++) {
    const int* indices = triangles[i].vIndices_;
    ss << "f " << indices[0]+1 << " " << indices[1]+1 << " " << indices[2]+1 << std::endl;
  }
}

void Files::savePLY(Mesh* mesh, std::ostream& ss) const {
  if (!ss) {
    return;
  }
  const TriangleVector &triangles = mesh->getTriangles();
  const VertexVector &vertices = mesh->getVertices();
  const float scale = 1/mesh->getScale();
  const int nbTriangles = triangles.size();
  const int nbVertices = vertices.size();

  // write header
  ss << "ply" << std::endl;
  ss << "format ascii 1.0" << std::endl;
  ss << "element vertex " << nbVertices << std::endl;
  ss << "property float x" << std::endl;
  ss << "property float y" << std::endl;
  ss << "property float z" << std::endl;
  ss << "property uchar red" << std::endl;
  ss << "property uchar green" << std::endl;
  ss << "property uchar blue" << std::endl;
  ss << "element face " << nbTriangles << std::endl;
  ss << "property list uchar uint vertex_indices" << std::endl;
  ss << "end_header" << std::endl;

  // write geometry
  for (int i=0; i<nbVertices; i++) {
    const Vector3 cur = scale*vertices[i];
    const Vector3& color = vertices[i].material_;
    const unsigned int red = static_cast<unsigned int>(255.0f * color.x());
    const unsigned int green = static_cast<unsigned int>(255.0f * color.y());
    const unsigned int blue = static_cast<unsigned int>(255.0f * color.z());
    ss << cur.x() << " " << cur.y() << " " << cur.z() << " " << red << " " << green << " " << blue << std::endl;
  }
  for (int i=0; i<nbTriangles; i++) {
    const int* indices = triangles[i].vIndices_;
    ss << "3 " << indices[0] << " " << indices[1] << " " << indices[2] << std::endl;
  }
}

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable : 4996) // 'scanf': This function or variable may be unsafe. Consider using scanf_s instead
#endif

Mesh* Files::loadOBJ(const std::string &filepath) const
{
  std::ifstream file(filepath.c_str());
  if(!file)
    return 0;
  Mesh *mesh = new Mesh();
  TriangleVector &triangles = mesh->getTriangles();
  VertexVector &vertices = mesh->getVertices();
  std::string line;
  int iVer1, iVer2, iVer3, iVer4; //vertex indices
  iVer4 = -1;
  float x, y, z;
  while(std::getline(file,line))
  {
    if(memcmp(line.c_str(),"v ", 2)==0) //vertex
    {
      std::stringstream ss;
      ss.str(line.substr(2));
      ss >> x;
      ss >> y;
      ss >> z;
      vertices.push_back(Vertex(x, y, z, vertices.size()));
    }
    else if(memcmp(line.c_str(),"f ",2)==0) //face
    {
      char* iVerts[4];
      for (int i=0; i<4; i++) {
        iVerts[i] = new char[line.size()];
      }
      iVerts[3][0] = '\0';
      sscanf(line.substr(2).c_str(), "%s %s %s %s", iVerts[0], iVerts[1], iVerts[2], iVerts[3]);
      sscanf(iVerts[0],"%d//", &iVer1);
      sscanf(iVerts[1],"%d//", &iVer2);
      sscanf(iVerts[2],"%d//", &iVer3);
      sscanf(iVerts[3],"%d//", &iVer4);
      if(iVer1<0)
      {
        iVer1 = vertices.size()+iVer1;
        iVer2 = vertices.size()+iVer2;
        iVer3 = vertices.size()+iVer3;
      }
      else
      {
        --iVer1;
        --iVer2;
        --iVer3;
      }
      Vertex &v1 = vertices[iVer1];
      Vertex &v2 = vertices[iVer2];
      Vertex &v3 = vertices[iVer3];
      v1.addTriangle(triangles.size());
      v2.addTriangle(triangles.size());
      v3.addTriangle(triangles.size());
      triangles.push_back(Triangle((v2-v1).cross(v3-v1).normalized(), iVer1, iVer2, iVer3, triangles.size()));
      //quad to triangle...
      if(iVer4!=-1)
      {
        if(iVer4<0)
          iVer4 = vertices.size() + iVer4;
        else
          --iVer4;
        Vertex &v4 = vertices[iVer4];
        v1.addTriangle(triangles.size());
        v3.addTriangle(triangles.size());
        v4.addTriangle(triangles.size());
        triangles.push_back(Triangle((v3-v1).cross(v4-v1).normalized(), iVer1, iVer3, iVer4, triangles.size()));
        iVer4 = -1;
      }
      for (int i=0; i<4; i++) {
        delete[] iVerts[i];
      }
    }
  }
  file.close();
  mesh->initMesh();
  return mesh;
}

#ifdef _WIN32
#pragma warning(pop)
#endif

/** Load 3DS file */
Mesh* Files::load3DS(const std::string &filepath) const
{
  std::ifstream file(filepath.c_str(), std::ios::binary);
  if(!file)
    return 0;
  Mesh *mesh = new Mesh();
  char charV;
  int i=0;
  unsigned int chunkLength;
  unsigned short chunkId, nbTotal, faceFlag;
  VertexVector &vertices = mesh->getVertices();
  TriangleVector &triangles = mesh->getTriangles();
  while(file.good())
  {
    file.read((char*)&chunkId,2);
    file.read((char*)&chunkLength,4);
    switch (chunkId)
    {
    case 0x4d4d: //main3DS
      break;
    case 0x3d3d: //edit3DS
      break;
    case 0x4000: //edit object
      i=0;
      do
      {
        file.read((char*)&charV,1);
        ++i;
      }while(charV != '\0');
      break;
    case 0x4100: //obj trimesh
      break;
    case 0x4110: //vertex
      file.read((char*)&nbTotal, sizeof(unsigned short));
      vertices.reserve(nbTotal);
      float x, y, z;
      for(i=0; i<nbTotal; ++i)
      {
        file.read((char*)&x,sizeof(float));
        file.read((char*)&y,sizeof(float));
        file.read((char*)&z,sizeof(float));
        vertices.push_back(Vertex(x,y,z,i));
      }
      break;
    case 0x4120: //face
      file.read((char*)&nbTotal, sizeof(unsigned short));
      unsigned short iVer1, iVer2, iVer3;
      triangles.reserve(nbTotal);
      for(i=0; i<nbTotal; ++i)
      {
        file.read((char*)&iVer1,sizeof(unsigned short));
        file.read((char*)&iVer2,sizeof(unsigned short));
        file.read((char*)&iVer3,sizeof(unsigned short));
        file.read((char*)&faceFlag,sizeof(unsigned short));
        Vertex &v1 = vertices[iVer1];
        Vertex &v2 = vertices[iVer2];
        Vertex &v3 = vertices[iVer3];
        v1.addTriangle(i);
        v2.addTriangle(i);
        v3.addTriangle(i);
        triangles.push_back(Triangle((v2-v1).cross(v3-v1).normalized(),iVer1,iVer2,iVer3,i));
      }
      break;
    case 0x4140: //text coord
      file.read((char*)&nbTotal, sizeof(unsigned short));
      float u, v;
      for(i=0; i<nbTotal; ++i)
      {
        file.read((char*)&u,sizeof(float));
        file.read((char*)&v,sizeof(float));
      }
      break;
    default:
      file.seekg(chunkLength-6, std::ios_base::cur);
    }
  }
  file.close();
  mesh->initMesh();
  mesh->moveTo(Vector3::Zero());
  return mesh;
}
