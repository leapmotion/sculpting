#include "FILES.h"
#include <stdlib.h>
#include <sstream>

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
    if (!file)
        return 0;
    Mesh *mesh = new Mesh();
    std::set<Vertex> verticesSet; //detect already read vertices ...
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
    mesh->initVBO();
    mesh->moveTo(Vector3::Zero());
    return mesh;
}

/** Check if the vertex already exists */
int Files::detectNewVertex(const Vertex &v, int iTri, std::set<Vertex> &verticesSet, VertexVector &vertices) const
{
    std::pair<std::set<Vertex>::iterator, bool> pair = verticesSet.insert(v);
    int iVert = (*pair.first).id_;
    if (pair.second)
        vertices.push_back(v);
    vertices[iVert].addTriangle(iTri);
    return iVert;
}

/** Save file in STL format */
void Files::saveSTL(Mesh *mesh, const std::string &filepath) const
{
    std::ofstream file(filepath.c_str(), std::ios::binary);
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

/** Load OBJ file */
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
            std::stringstream ss;
            ss.str(line.substr(2));
            ss >> iVer1;
            ss >> iVer2;
            ss >> iVer3;
            ss >> iVer4;
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
        }
    }
    file.close();
    mesh->initMesh();
    mesh->initVBO();
    mesh->moveTo(Vector3::Zero());
    return mesh;
}

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
    mesh->initVBO();
    mesh->moveTo(Vector3::Zero());
    return mesh;
}
