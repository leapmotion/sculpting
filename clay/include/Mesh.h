#ifndef __MESH_H__
#define __MESH_H__

#include <cinder/gl/gl.h>
#include "DataTypes.h"
#include <string>
#include <algorithm>
#include <stdint.h>
#include <list>
#include "Tools.h"
#include "Triangle.h"
#include "Vertex.h"
#include "State.h"
#include "omp.h"
#include "Geometry.h"

class Octree;

/**
* Mesh
* @author Stéphane GINIER
*/
class Mesh
{
public:
	static const float globalScale_; //for precision issue...
	static int stateMask_; //for history

public:
	Mesh();
	~Mesh();
	TriangleVector& getTriangles();
	VertexVector& getVertices();
	std::vector<Octree*>& getLeavesUpdate();
	Triangle& getTriangle(int i);
	Vertex& getVertex(int i);
	int getNbTriangles() const;
	int getNbVertices() const;
	Vector3 getCenter() const;
	Octree* getOctree() const;
	float getScale() const;
	void setIsSelected(bool);
	bool getDisplayOctree() const;
	void toggleDisplayOctree();
	Matrix4x4& getTransformation();
	Matrix4x4 getInverseTransformation() const;

	std::vector<int> getTrianglesFromVertices(const std::vector<int> &iVerts);
	std::vector<int> getVerticesFromTriangles(const std::vector<int> &iTris);
	void expandTriangles(std::vector<int> &iTris, int nRing);
	void expandVertices(std::vector<int> &iVerts, int nRing);
	void computeRingVertices(int iVert);
	void getVerticesInsideSphere(const Vector3& point, float radiusWorldSquared, std::vector<int>& result);

	Vector3 getTriangleCenter(int iTri) const;
	void moveTo(const Vector3& destination);
	void setTransformation(const Matrix4x4& matTransform);

	void draw(GLint vertex, GLint normal);
	void initVBO();
	void initMesh();

	void updateMesh(const std::vector<int> &iTris, const std::vector<int> &iVerts);

	std::vector<int> subdivide(std::vector<int> &iTris,std::vector<int> &iVerts,float inradiusMaxSquared);
	void triangleSubdivision(int iTri);

	void checkLeavesUpdate();

	//undo-redo
	TriangleVector& getTrianglesState();
	VertexVector& getVerticesState();
	void startPushState();
	void pushState(const std::vector<int> &iTris, const std::vector<int> &iVerts);
	void undo();
	void redo();
	void recomputeOctree(const Aabb &aabbSplit);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

	class GLBuffer {
	public:
		GLBuffer(GLenum type) : type_(type), buffer_(0) { }
		void create() {
			glGenBuffers(1, &buffer_);
			checkError();
		}
		void setUsagePattern(GLenum pattern) {
			pattern_ = pattern;
		}
		void bind() {
			glBindBuffer(type_, buffer_);
			checkError();
		}
		void allocate(const void* data, int count) {
			glBufferData(type_, count, data, pattern_);
			checkError();
		}
		void release() {
			glBindBuffer(type_, 0);
			checkError();
		}
		int size() const {
			GLint value = -1;
			glGetBufferParameteriv(type_, GL_BUFFER_SIZE, &value);
			checkError();
			return value;
		}
		void* map(GLuint access) {
			void* ptr = glMapBufferARB(type_, access);
			if (ptr == NULL) {
				std::cout << "asdf" << std::endl;
			}
			checkError();
			return ptr;
		}
		bool unmap() {
			bool result = glUnmapBufferARB(type_) == GL_TRUE;
			checkError();
			return result;
		}
		bool isCreated() const {
			return buffer_ != 0;
		}
		void destroy() {
			glDeleteBuffers(1, &buffer_);
		}
		static void checkError() {
			GLenum err = glGetError();
			if (err != GL_NO_ERROR) {
				std::cout << err << std::endl;
			}
		}
	private:
		GLenum pattern_;
		GLuint buffer_;
		GLenum type_;
	};

	void updateOctree(const std::vector<int> &iTris);
	void updateNormals(const std::vector<int> &iVerts);
	float angleTri(int iTri, int iVer);
	void updateVertexBuffer(const std::vector<int> &iVerts);
	void updateNormalBuffer(const std::vector<int> &iVerts);
	void updateIndexBuffer(const std::vector<int> &iTris);
	void updateTransformation();

	VertexVector vertices_; //vertices
	TriangleVector triangles_; //triangles
	GLBuffer verticesBuffer_; //vertices buffer (openGL)
	GLBuffer normalsBuffer_; //normals buffer (openGL)
	GLBuffer indicesBuffer_; //indexes (openGL)
	Vector3 center_; //center of mesh
	float scale_; //scale
	Octree *octree_; //octree
	bool displayOctree_; //if the octree is displayed
	Matrix4x4 matTransform_; //transformation matrix of the mesh
	GLfloat matTransformArray_[16]; //transformation matrix of the mesh (openGL)
	std::vector<Octree*> leavesUpdate_; //leaves of the octree to check

	//undo-redo
	std::list<State> undo_; //undo actions
	std::list<State> redo_; //redo actions
	std::list<State>::iterator undoIte_; //iterator to undo
	bool beginIte_; //end of undo action
};

#endif /*__MESH_H__*/
