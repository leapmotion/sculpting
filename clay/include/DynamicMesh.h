#ifndef __DYNAMICMESH_H__
#define __DYNAMICMESH_H__

#include "cinder/Vector.h"
using namespace cinder;

#include <list>
#include <map>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <assert.h>
#include <stdio.h>


// *** forward declarations ***
class DynamicMeshHalfedge; 
class DynamicMesh;

/*****************************************************
   Dynamic vertex class
******************************************************/
class DynamicMeshVertex
{
private:

	Vec3f _pos; // position
	Vec3f _nrm; // normal
	DynamicMeshHalfedge* _halfedge; // pointer to just one of its connected halfedges
	bool _dead; // flag for deletion
	bool _dirty; // flag to recalculate stuff

public:

	DynamicMeshVertex() : _halfedge(0), _dead(false), _dirty(true) { }
	inline DynamicMeshHalfedge* getHalfedge() { return _halfedge; }
	inline void setHalfEdge(DynamicMeshHalfedge* _Halfedge) { _halfedge = _Halfedge; }
	inline const Vec3f& getPosition() const { return _pos; }
	inline void setPosition(const Vec3f& _Pos) { _pos = _Pos; _dirty = true; }
	inline void modifyPosition(const Vec3f& _DPos) { _pos += _DPos; _dirty = true; }
	inline const Vec3f& getNormal() const { return _nrm; }
	inline void setNormal(const Vec3f& _Nrm) { _nrm = _Nrm; _dirty = true; }
	inline void modifyNormal(const Vec3f& _DNrm) { _nrm += _DNrm; _dirty = true; }
	inline bool isDead() const { return _dead; }
	inline void setDead() { _dead = true; }
	inline bool isDirty() const { return _dirty; }
	inline void setDirty(const bool _Dirty) { _dirty = _Dirty; }
	void calculateNormal();
	void smooth(const float _Strength);
};

/*****************************************************
   Dynamic halfedge class
******************************************************/
class DynamicMeshHalfedge
{
private:

	DynamicMeshVertex* _back; // vertex halfedge is pointing from
	DynamicMeshHalfedge* _next; // next halfedge in face
	DynamicMeshHalfedge* _twin; // other half of edge
	bool _dead; // flag for deletion
	bool _processed; // flag for processing

public:

	DynamicMeshHalfedge() : _back(0), _next(0), _twin(0), _dead(false) { }

	inline void setBack(DynamicMeshVertex* _Vert) { _back = _Vert; _back->setHalfEdge(this); }
	inline void setNext(DynamicMeshHalfedge* _Next) { _next = _Next; }
	inline void setTwin(DynamicMeshHalfedge* _Twin) { _twin = _Twin; }
/*
	inline void joinTwin(DynamicMeshHalfedge* _Twin) 
	{ 
		if( _twin && _twin->Twin()==this ) _twin->setTwin(0); // disconnect from current twin
		setTwin(_Twin); // set new twin
		if( _twin ) _twin->setTwin(this); // conect to new twin
	}
*/
	inline DynamicMeshHalfedge* Next() { return _next; }
	inline DynamicMeshHalfedge* Prev() { return _next->Next(); }
	inline DynamicMeshHalfedge* Twin() { return _twin; }

	inline DynamicMeshVertex* Back() { return _back; }
	inline DynamicMeshVertex* Front() { return _next->Back(); }
	inline DynamicMeshVertex* Left() { return Prev()->Back(); }
	inline DynamicMeshVertex* Right() { return (_twin) ? _twin->Prev()->Back() : 0; }

	inline bool isDead() const { return _dead; }
	inline void setDead() { _dead = true; }
	inline bool getProcessed() const { return _processed; }
	inline void setProcessed(const bool _Processed) { _processed = _Processed; }

	void collapse(DynamicMesh* _Mesh);
	void split(DynamicMesh* _Mesh); // needs to add new stuff to mesh
	void rotate(DynamicMesh* _Mesh);
};


/*****************************************************
   Dynamic mesh class
   Uses winged edge structure for quick operations
   such as splitting edges
******************************************************/
class DynamicMesh
{
public:

	friend class DynamicMeshVertex;
	friend class DynamicMeshHalfedge;

	typedef std::list<DynamicMeshVertex*> VertexList;
	typedef VertexList::iterator VertexIterator;
	typedef std::list<DynamicMeshHalfedge*> HalfedgeList;
	typedef HalfedgeList::iterator HalfedgeIterator;
	typedef HalfedgeList::reverse_iterator HalfedgeReverseIterator;

	// *** public methods ***
	DynamicMesh();
	~DynamicMesh();
	VertexList& getVertices() { return _vertices; }
	HalfedgeList& getHalfedges() { return _halfedges; }
	void clear();
	inline bool isDirty() const { return _dirty; }
	inline void setDirty(const bool _Dirty) { _dirty = _Dirty; }
	DynamicMeshVertex* addVertex(const Vec3f& _Pos,const Vec3f& _Nrm=Vec3f(0,0,0));
	DynamicMeshHalfedge* addHalfEdge(DynamicMeshVertex* _Vert,DynamicMeshHalfedge* _Next=0,DynamicMeshHalfedge* _Twin=0);
	void deleteDead();
	void calculateAllNormals();
	bool isTriangular();
	bool isManifold();
	void makeManifold();
	void createTetrahedron(const Vec3f& _Pos,const float _Radius);
	void collapseShortEdges(const float _LowerThreshold);
	void splitLongEdges(const float _UpperThreshold);
	void flipLongFaces();
	void refineMesh(const float _LowerThreshold,const float _UpperThreshold);
	void refineWholeMesh(const float _LowerThreshold,const float _UpperThreshold);
	void selfIntersect(); // in separate cpp file
	void draw();
	void drawEdges();
	void drawNormals(const float _Length=1.f);
	int loadOBJFile(const std::string& _Filename);
	int saveOBJFile(const std::string& _Filename);

private:

	// *** private members ***
	VertexList _vertices;
	HalfedgeList _halfedges;
	int _num_vertices_to_be_deleted;
	int _num_halfedges_to_be_deleted;
	bool _dirty;

};

#endif // #ifndef __DYNAMICMESH_H__
