#include "Mesh.h"
#include "Octree.h"
#include <iostream>

const float Mesh::globalScale_ = 500.f;
int Mesh::stateMask_ = 1;
const int undoLimit_ = 10;

/** Constructor */
Mesh::Mesh() : center_(Vector3::Zero()), scale_(1),
	octree_(0), matTransform_(Matrix4x4::Identity()), beginIte_(false), verticesBuffer_(GL_ARRAY_BUFFER),
  normalsBuffer_(GL_ARRAY_BUFFER), indicesBuffer_(GL_ELEMENT_ARRAY_BUFFER), colorsBuffer_(GL_ARRAY_BUFFER)
{
	updateTransformation();
}

/** Destructor */
Mesh::~Mesh()
{
	delete octree_;
}

/** Setters/Getters */
TriangleVector& Mesh::getTriangles() { return triangles_; }
VertexVector& Mesh::getVertices() { return vertices_; }
std::vector<Octree*>& Mesh::getLeavesUpdate() { return leavesUpdate_; }
Triangle& Mesh::getTriangle(int i) { return triangles_[i]; }
Vertex& Mesh::getVertex(int i) { return vertices_[i]; }
int Mesh::getNbTriangles() const { return triangles_.size(); }
int Mesh::getNbVertices() const { return vertices_.size(); }
Vector3 Mesh::getCenter() const { return center_; }
Octree* Mesh::getOctree() const { return octree_; }
float Mesh::getScale() const { return scale_; }
Matrix4x4& Mesh::getTransformation() { return matTransform_; }
Matrix4x4 Mesh::getInverseTransformation() const { return matTransform_.inverse(); }

/** Return all the triangles linked to a group of vertices */
std::vector<int> Mesh::getTrianglesFromVertices(const std::vector<int> &iVerts)
{
	++Triangle::tagMask_;
	std::vector<int> triangles;
	int nbVerts = iVerts.size();
	for(int i=0;i<nbVerts;++i)
	{
		std::vector<int> &iTris = vertices_[iVerts[i]].tIndices_;
		int nbTris = iTris.size();
		for(int j=0;j<nbTris;++j)
		{
			int iTri = iTris[j];
			if(triangles_[iTri].tagFlag_!=Triangle::tagMask_)
			{
				triangles.push_back(iTri);
				triangles_[iTri].tagFlag_=Triangle::tagMask_;
			}
		}
	}
	return triangles;
}

/** Return all the triangles linked to a group of vertices */
std::vector<int> Mesh::getVerticesFromTriangles(const std::vector<int> &iTris)
{
	++Vertex::tagMask_;
	std::vector<int> vertices;
	int nbTris = iTris.size();
	for(int i=0;i<nbTris;++i)
	{
		Triangle &t=triangles_[iTris[i]];
		int iVer1 = t.vIndices_[0];
		int iVer2 = t.vIndices_[1];
		int iVer3 = t.vIndices_[2];
		if(vertices_[iVer1].tagFlag_!=Vertex::tagMask_)
		{
			vertices.push_back(iVer1);
			vertices_[iVer1].tagFlag_=Vertex::tagMask_;
		}
		if(vertices_[iVer2].tagFlag_!=Vertex::tagMask_)
		{
			vertices.push_back(iVer2);
			vertices_[iVer2].tagFlag_=Vertex::tagMask_;
		}
		if(vertices_[iVer3].tagFlag_!=Vertex::tagMask_)
		{
			vertices.push_back(iVer3);
			vertices_[iVer3].tagFlag_=Vertex::tagMask_;
		}
	}
	return vertices;
}

/** Get more triangles (n-ring) */
void Mesh::expandTriangles(std::vector<int> &iTris, int nRing)
{
	++Triangle::tagMask_;
	int nbTris = iTris.size();
	for(int i=0;i<nbTris;++i)
		triangles_[iTris[i]].tagFlag_ = Triangle::tagMask_;
	int iBegin = 0;
	while(nRing)
	{
		--nRing;
		for(int i=iBegin;i<nbTris;++i)
		{
			Triangle &t=triangles_[iTris[i]];
			std::vector<int> &iTris1 = vertices_[t.vIndices_[0]].tIndices_;
			std::vector<int> &iTris2 = vertices_[t.vIndices_[1]].tIndices_;
			std::vector<int> &iTris3 = vertices_[t.vIndices_[2]].tIndices_;
			int nbTris1 = iTris1.size();
			int nbTris2 = iTris2.size();
			int nbTris3 = iTris3.size();
			for(int j=0;j<nbTris1;++j)
			{
				Triangle &t = triangles_[iTris1[j]];
				if(t.tagFlag_!=Triangle::tagMask_)
				{
					t.tagFlag_ = Triangle::tagMask_;
					iTris.push_back(iTris1[j]);
				}
			}
			for(int j=0;j<nbTris2;++j)
			{
				Triangle &t = triangles_[iTris2[j]];
				if(t.tagFlag_!=Triangle::tagMask_)
				{
					t.tagFlag_ = Triangle::tagMask_;
					iTris.push_back(iTris2[j]);
				}
			}
			for(int j=0;j<nbTris3;++j)
			{
				Triangle &t = triangles_[iTris3[j]];
				if(t.tagFlag_!=Triangle::tagMask_)
				{
					t.tagFlag_ = Triangle::tagMask_;
					iTris.push_back(iTris3[j]);
				}
			}
		}
		iBegin = nbTris;
		nbTris = iTris.size();
	}
}

/** Get more vertices (n-ring) */
void Mesh::expandVertices(std::vector<int> &iVerts, int nRing)
{
	++Vertex::tagMask_;
	int nbVerts = iVerts.size();
	for(int i=0;i<nbVerts;++i)
		vertices_[iVerts[i]].tagFlag_ = Vertex::tagMask_;
	int iBegin = 0;
	while(nRing)
	{
		--nRing;
		for(int i=iBegin;i<nbVerts;++i)
		{
			std::vector<int> &ring = vertices_[iVerts[i]].ringVertices_;
			int nbRing = ring.size();
			for(int j=0;j<nbRing;++j)
			{
				Vertex &vRing = vertices_[ring[j]];
				if(vRing.tagFlag_!=Vertex::tagMask_)
				{
					vRing.tagFlag_ = Vertex::tagMask_;
					iVerts.push_back(ring[j]);
				}
			}
		}
		iBegin = nbVerts;
		nbVerts = iVerts.size();
	}
}

/** Compute the vertices around a vertex */
void Mesh::computeRingVertices(int iVert)
{
	++Vertex::tagMask_;
	std::vector<int> &iTris = vertices_[iVert].tIndices_;
	std::vector<int> &ring = vertices_[iVert].ringVertices_;
	ring.clear();
	int nbTris = iTris.size();
	for(int i=0;i<nbTris;++i)
	{
		const Triangle &t=triangles_[iTris[i]];
		int iVer1 = t.vIndices_[0];
		int iVer2 = t.vIndices_[1];
		int iVer3 = t.vIndices_[2];
		if(iVer1!=iVert && vertices_[iVer1].tagFlag_!=Vertex::tagMask_)
		{
			ring.push_back(iVer1);
			vertices_[iVer1].tagFlag_=Vertex::tagMask_;
		}
		if(iVer2!=iVert && vertices_[iVer2].tagFlag_!=Vertex::tagMask_)
		{
			ring.push_back(iVer2);
			vertices_[iVer2].tagFlag_=Vertex::tagMask_;
		}
		if(iVer3!=iVert && vertices_[iVer3].tagFlag_!=Vertex::tagMask_)
		{
			ring.push_back(iVer3);
			vertices_[iVer3].tagFlag_=Vertex::tagMask_;
		}
	}
}

void Mesh::getVerticesInsideSphere(const Vector3& point, float radiusWorldSquared, std::vector<int>& result) {
	VertexVector &vertices = getVertices();
	std::vector<Octree*> &leavesHit = getLeavesUpdate();
	std::vector<int> iTrisInCells = getOctree()->intersectSphere(point,radiusWorldSquared,leavesHit);
	std::vector<int> iVerts = getVerticesFromTriangles(iTrisInCells);
	int nbVerts = iVerts.size();
	++Vertex::sculptMask_;
	for (int i=0;i<nbVerts;++i)
	{
		Vertex &v=vertices[iVerts[i]];
		const float distSquared = (v-point).squaredNorm();
		if(distSquared<radiusWorldSquared)
		{
			v.sculptFlag_ = Vertex::sculptMask_;
			result.push_back(iVerts[i]);
		}
	}
}

void Mesh::getVerticesInsideBrush(const Brush& brush, std::vector<int>& result) {
  VertexVector &vertices = getVertices();
  std::vector<Octree*> &leavesHit = getLeavesUpdate();
  std::vector<int> iTrisInCells = getOctree()->intersectSphere(brush.boundingSphereCenter(),brush.boundingSphereRadiusSq(),leavesHit);
	std::vector<int> iVerts = getVerticesFromTriangles(iTrisInCells);
	int nbVerts = iVerts.size();
	++Vertex::sculptMask_;
	for (int i=0;i<nbVerts;++i)
	{
		Vertex &v=vertices[iVerts[i]];
    if (brush.contains(v)) {
      v.sculptFlag_ = Vertex::sculptMask_;
      result.push_back(iVerts[i]);
    }
	}
}

/** Return center of a triangle */
Vector3 Mesh::getTriangleCenter(int iTri) const
{
	const Triangle &t=triangles_[iTri];
	return (vertices_[t.vIndices_[0]]+vertices_[t.vIndices_[1]]+vertices_[t.vIndices_[2]])/3;
}

/** Move the mesh center to a certain point */
void Mesh::moveTo(const Vector3& destination)
{
	matTransform_ = Tools::translationMatrix(destination-center_);
	updateTransformation();
}

/** Set transformation */
void Mesh::setTransformation(const Matrix4x4& matTransform)
{
	matTransform_=matTransform;
	updateTransformation();
}

/** Update transformation */
void Mesh::updateTransformation()
{
	float *dataMat = matTransform_.data();
	for (int i= 0; i < 16; ++i)
		matTransformArray_[i] = dataMat[i];
}

void Mesh::draw(GLint vertex, GLint normal, GLint color) {
	verticesBuffer_.bind();
	glEnableVertexAttribArray(vertex);
	GLBuffer::checkError();
	glVertexAttribPointer(vertex, 3, GL_FLOAT, GL_TRUE, 0, 0);
	GLBuffer::checkError();

	normalsBuffer_.bind();
	glEnableVertexAttribArray(normal);
	GLBuffer::checkError();
	glVertexAttribPointer(normal, 3, GL_FLOAT, GL_TRUE, 0, 0);
	GLBuffer::checkError();

  colorsBuffer_.bind();
  glEnableVertexAttribArray(color);
  GLBuffer::checkError();
  glVertexAttribPointer(color, 3, GL_FLOAT, GL_TRUE, 0, 0);
  GLBuffer::checkError();

	indicesBuffer_.bind();
	glDrawElements(GL_TRIANGLES, getNbTriangles()*3, GL_UNSIGNED_INT, 0);
	indicesBuffer_.release();
	glDisableVertexAttribArray(vertex);
	glDisableVertexAttribArray(normal);
  glDisableVertexAttribArray(color);

	normalsBuffer_.release();
	verticesBuffer_.release();
  colorsBuffer_.release();
}

void Mesh::drawOctree() const {
	glPushMatrix();
	glMultMatrixf(matTransformArray_);
	octree_->draw();
	glPopMatrix();
}

/** Initialize Vertex Buffer Object (VBO) */
void Mesh::initVBO()
{
	const int nbVertices = getNbVertices();
	const int nbTriangles = getNbTriangles();
	const int verticesBytes = nbVertices*3*sizeof(GLfloat)*2;
	const int indicesBytes = nbTriangles*3*sizeof(GLuint)*2;

	GLfloat* verticesArray=(GLfloat*)malloc(verticesBytes);
	GLfloat* normalsArray=(GLfloat*)malloc(verticesBytes);
  GLfloat* colorsArray=(GLfloat*)malloc(verticesBytes);
	GLuint* indicesArray=(GLuint*)malloc(indicesBytes);

#pragma omp parallel for
	for (int i=0;i<nbVertices;++i)
	{
		Vertex &ver=vertices_[i];
		verticesArray[i*3]=ver.x();
		verticesArray[i*3+1]=ver.y();
		verticesArray[i*3+2]=ver.z();
		const std::vector<int> &iTri=ver.tIndices_;
		int nbTri = iTri.size();
    const Vector3& normal = ver.normal_;
		normalsArray[i*3]=normal.x();
		normalsArray[i*3+1]=normal.y();
		normalsArray[i*3+2]=normal.z();
    const Vector3& color = ver.material_;
    colorsArray[i*3]=color.x();
    colorsArray[i*3+1]=color.y();
    colorsArray[i*3+2]=color.z();
	}

#pragma omp parallel for
	for (int i=0;i<nbTriangles;++i)
	{
		Triangle &tri=triangles_[i];
		indicesArray[i*3]=tri.vIndices_[0];
		indicesArray[i*3+1]=tri.vIndices_[1];
		indicesArray[i*3+2]=tri.vIndices_[2];
	}

	if (verticesBuffer_.isCreated()) {
		verticesBuffer_.destroy();
	}
	verticesBuffer_.create();
	verticesBuffer_.bind();
	verticesBuffer_.allocate(verticesArray, verticesBytes, GL_DYNAMIC_DRAW);
	verticesBuffer_.release();

	if (normalsBuffer_.isCreated()) {
		normalsBuffer_.destroy();
	}
	normalsBuffer_.create();
	normalsBuffer_.bind();
	normalsBuffer_.allocate(normalsArray, verticesBytes, GL_DYNAMIC_DRAW);
	normalsBuffer_.release();

  if (colorsBuffer_.isCreated()) {
		colorsBuffer_.destroy();
	}
	colorsBuffer_.create();
	colorsBuffer_.bind();
  colorsBuffer_.allocate(colorsArray, verticesBytes, GL_DYNAMIC_DRAW);
	colorsBuffer_.release();

	if (indicesBuffer_.isCreated()) {
		indicesBuffer_.destroy();
	}
	indicesBuffer_.create();
	indicesBuffer_.bind();
  indicesBuffer_.allocate(indicesArray, indicesBytes, GL_DYNAMIC_DRAW);
	indicesBuffer_.release();

  free(verticesArray);
  free(normalsArray);
  free(colorsArray);
  free(indicesArray);
}

/** Initialize the mesh information : center, octree, scale ... */
void Mesh::initMesh()
{
	int nbVertices = getNbVertices();
	int nbTriangles = getNbTriangles();
	Aabb aabb;
	aabb.min_ = vertices_[0];
	aabb.max_ = vertices_[0];
	for(int i=0;i<nbVertices;++i)
	{
		computeRingVertices(i);
		aabb.expand(vertices_[i]);
	}
	center_ = aabb.getCenter();
	float diag = (aabb.max_-aabb.min_).norm();
	scale_ = Mesh::globalScale_/diag;
#pragma omp parallel for
	for(int i=0;i<nbVertices;++i)
		vertices_[i] = scale_*(vertices_[i] - center_);
  aabb.min_ -= center_;
  aabb.max_ -= center_;
	matTransform_ = Tools::scaleMatrix(scale_)*matTransform_;
	aabb.max_*=scale_;
	aabb.min_*=scale_;
	center_*=scale_;
#pragma omp parallel for
	for(int i=0;i<nbTriangles;++i)
	{
		Triangle &t = triangles_[i];
		t.aabb_ = Geometry::computeTriangleAabb(vertices_[t.vIndices_[0]],vertices_[t.vIndices_[1]],vertices_[t.vIndices_[2]]);
	}
	aabb.checkFlat((aabb.max_-aabb.min_).norm()*0.02f);
	Vector3 vecShift = (aabb.max_-aabb.min_)*0.2f; //root octree bigger than minimum aabb...
	aabb.min_-=vecShift;
	aabb.max_+=vecShift;
	std::vector<int> triangles(nbTriangles);
#pragma omp parallel for
	for (int i=0;i<nbTriangles;++i)
		triangles[i] = i;
	++Triangle::tagMask_;
	if(octree_)
		delete octree_;
	octree_ = new Octree();
	octree_->build(this,triangles,aabb);
  for (int i=0;i<nbVertices;++i) {
    Vertex &ver=vertices_[i];
		const std::vector<int> &iTri=ver.tIndices_;
		int nbTri = iTri.size();
		Vector3 normal(Vector3::Zero());
		for (int j=0;j<nbTri;++j) {
			normal+=triangles_[iTri[j]].normal_;
		}
    float length = normal.norm();
    if (length < 0.0001f) {
      // normals added up to zero length, so just pick one
      normal = triangles_[iTri[0]].normal_;
    } else {
      normal = normal/length;
    }
    assert(fabs(normal.squaredNorm() - 1.0f) < 0.001f);
		ver.normal_=normal;
  }
}

/** Update geometry  */
void Mesh::updateMesh(const std::vector<int> &iTris, const std::vector<int> &iVerts)
{
	indicesBuffer_.bind();
	int nbIndicesBuffer = indicesBuffer_.size() / (sizeof(GLuint)*3);
	indicesBuffer_.release();
	verticesBuffer_.bind();
	int nbVerticesBuffer = verticesBuffer_.size() / (sizeof(GLfloat)*3);
	verticesBuffer_.release();
	int nbTris=iTris.size();
#pragma omp parallel for //recompute triangle normals and aabb
	for (int i=0;i<nbTris;++i)
	{
		Triangle &t=triangles_[iTris[i]];
		const Vector3& v1=vertices_[t.vIndices_[0]];
		const Vector3& v2=vertices_[t.vIndices_[1]];
		const Vector3& v3=vertices_[t.vIndices_[2]];
    Vector3 normal = (v2-v1).cross(v3-v1);
    float length = normal.norm();
    if (length < 0.001f) {
      t.normal_ = Vector3::UnitY();
    } else {
      t.normal_ = normal/length;
    }
    assert(fabs(t.normal_.norm() - 1.0f) < 0.001f);
		t.aabb_ = Geometry::computeTriangleAabb(v1,v2,v3);
	}
	updateOctree(iTris);
	updateNormals(iVerts);
	assert(_CrtCheckMemory());
  if(nbIndicesBuffer>getNbTriangles() && nbVerticesBuffer>getNbVertices()) {  
	  updateIndexBuffer(iTris);
	  updateVertexBuffer(iVerts);
	  updateNormalBuffer(iVerts);
    updateColorBuffer(iVerts);
  }
	else
	{
		initVBO();
	}
}

/**
* Update Octree
* For each triangle we check if its position inside the octree has changed
* if so... we mark this triangle and we remove it from its former cells
* We push back the marked triangles into the octree
*/
void Mesh::updateOctree(const std::vector<int> &iTris)
{
	int nbTris = iTris.size();
	std::vector<int> trisToMove;
	for (int i=0;i<nbTris;++i) //recompute position inside the octree
	{
		Triangle &t=triangles_[iTris[i]];
		Octree *leaf=t.leaf_;
		if(!leaf->getAabbSplit().pointInside(t.aabb_.getCenter()))
		{
			trisToMove.push_back(iTris[i]);
			std::vector<int> &trisLeaf = leaf->getTriangles();
			if(trisLeaf.size()>0)
			{
				int iTriLast = trisLeaf.back();
				int iPos = t.posInLeaf_;
				trisLeaf[iPos] = iTriLast;
				triangles_[iTriLast].posInLeaf_ = iPos;
				trisLeaf.pop_back();
			}
		}
		else if(!t.aabb_.isInside(leaf->getAabbLoose())) {
			leaf->getAabbLoose().expand(t.aabb_);
    }
	}
	int nbTrisToMove = trisToMove.size();
	for(int i=0;i<nbTrisToMove;++i) //add triangle to the octree
	{
		Triangle &tri = triangles_[trisToMove[i]];
		if(octree_->getAabbLoose().isOutside(tri.aabb_)) //we reconstruct the whole octree, slow... but rare
		{
			Aabb aabb(octree_->getAabbSplit().min_,octree_->getAabbSplit().max_);
			delete octree_;
			Vector3 vecShift = (aabb.max_-aabb.min_)*0.2f;
			aabb.min_-=vecShift;
			aabb.max_+=vecShift;
			std::vector<int> triangles;
			for (int i=0;i<getNbTriangles();++i)
				triangles.push_back(i);
			octree_ = new Octree();
			++Triangle::tagMask_;
			octree_->build(this, triangles, aabb );
			leavesUpdate_.clear();
			break;
		}
		else
		{
			Octree* leaf=tri.leaf_;
			octree_->addTriangle(this,tri);
			if(leaf==tri.leaf_)
			{
				std::vector<int> &trisLeaf = leaf->getTriangles();
				tri.posInLeaf_=trisLeaf.size();
				trisLeaf.push_back(trisToMove[i]);
			}
		}
	}
}

/** Update normals */
void Mesh::updateNormals(const std::vector<int> &iVerts)
{
	int nbVers = iVerts.size();
#pragma omp parallel for
	for (int i=0;i<nbVers;++i)
	{
		Vertex &vert=vertices_[iVerts[i]];
		const std::vector<int> &iTri = vert.tIndices_;
		int nbTri = iTri.size();
		Vector3 normal(Vector3::Zero());
		for (int j=0;j<nbTri;++j)
			normal+=triangles_[iTri[j]].normal_;
    float length = normal.norm();
    if (length < 0.0001f) {
      // normals added up to zero length, so just pick one
      normal = triangles_[iTri[0]].normal_;
    } else {
      normal = normal/length;
    }
		vert.normal_ = normal;
    assert(fabs(vert.normal_.squaredNorm() - 1.0f) < 0.001f);
	}
}

/** Update vertex buffer */
void Mesh::updateVertexBuffer(const std::vector<int> &iVerts)
{
	int nbVerts=iVerts.size();
	GLfloat* verticesArray;
	verticesBuffer_.bind();
	verticesArray = (GLfloat*)verticesBuffer_.map(GL_WRITE_ONLY);
#pragma omp parallel for
	for(int i=0;i<nbVerts;++i)
	{
		int j=iVerts[i];
		Vertex &v = vertices_[j];
		verticesArray[j*3] = v.x();
		verticesArray[j*3+1] = v.y();
		verticesArray[j*3+2] = v.z();
	}
	verticesBuffer_.unmap();
	verticesBuffer_.release();
}

/** Update normal buffer */
void Mesh::updateNormalBuffer(const std::vector<int> &iVerts)
{
	int nbVerts=iVerts.size();
	GLfloat* normalsArray;
	normalsBuffer_.bind();
	normalsArray = (GLfloat*)normalsBuffer_.map(GL_WRITE_ONLY);
#pragma omp parallel for
	for(int i=0;i<nbVerts;++i)
	{
		int j=iVerts[i];
		Vector3& n = vertices_[j].normal_;
		normalsArray[j*3] = n.x();
		normalsArray[j*3+1] = n.y();
		normalsArray[j*3+2] = n.z();
	}
	normalsBuffer_.unmap();
	normalsBuffer_.release();
}

void Mesh::updateColorBuffer(const std::vector<int> &iVerts)
{
	int nbVerts=iVerts.size();
	GLfloat* colorsArray;
	colorsBuffer_.bind();
	colorsArray = (GLfloat*)colorsBuffer_.map(GL_WRITE_ONLY);
#pragma omp parallel for
	for(int i=0;i<nbVerts;++i)
	{
		int j=iVerts[i];
    const Vector3& color = vertices_[j].material_;
		colorsArray[j*3] = color.x();
		colorsArray[j*3+1] = color.y();
		colorsArray[j*3+2] = color.z();
	}
	colorsBuffer_.unmap();
	colorsBuffer_.release();
}

/** Update index buffer */
void Mesh::updateIndexBuffer(const std::vector<int> &iTris)
{
	int nbTris = iTris.size();
	GLuint* indicesArray;
	indicesBuffer_.bind();
	indicesArray = (GLuint*)indicesBuffer_.map(GL_WRITE_ONLY);
#pragma omp parallel for
	for(int i=0;i<nbTris;++i)
	{
		int j = iTris[i];
		Triangle &tri=triangles_[j];
		indicesArray[j*3]=tri.vIndices_[0];
		indicesArray[j*3+1]=tri.vIndices_[1];
		indicesArray[j*3+2]=tri.vIndices_[2];
	}
	indicesBuffer_.unmap();
	indicesBuffer_.release();
}

/** End of stroke, update octree (cut empty leaves or go deeper if needed) */
void Mesh::checkLeavesUpdate()
{
	Tools::tidy(leavesUpdate_);
	int nbLeaves = leavesUpdate_.size();
	std::vector<Octree*> cutLeaves;
	++Triangle::tagMask_;
	for(int i=0;i<nbLeaves;++i)
	{
		Octree* leaf = leavesUpdate_[i];
		if(!leaf)
			break;
		if(!leaf->getTriangles().size())
			Octree::checkEmptiness(leaf,cutLeaves);
		else if((int)leaf->getTriangles().size() > Octree::maxTriangles_ && leaf->getDepth() < Octree::maxDepth_)
			leaf->constructCells(this);
	}
	Tools::tidy(cutLeaves);
	int nbCutLeaves = cutLeaves.size();
	for(int i=0;i<nbCutLeaves;++i)
	{
		Octree *oc = cutLeaves[i];
		Octree **child = oc->getChildren();
		for(int j=0;j<8;++j)
			child[j]=0;
	}
	leavesUpdate_.clear();
}

/**
*****************************************
*****************************************
*****************************************
*****************************************
************* UNDO/REDO *****************
*****************************************
*****************************************
*****************************************
*****************************************
*/

/** Undo setters/Getters */
TriangleVector& Mesh::getTrianglesState() { return undoIte_->tState_; }
VertexVector& Mesh::getVerticesState() { return undoIte_->vState_; }

/** Start push state */
void Mesh::startPushState()
{
	++Mesh::stateMask_;
	if(beginIte_) {
		undo_.clear();
  } else if(undo_.size()>undoLimit_) {
		undo_.pop_front();
  }
	beginIte_ = false;
	redo_.clear();
	if(undo_.size())
	{
		std::list<State>::iterator lastIte = undo_.end();
		--lastIte;
		while(undoIte_!=lastIte)
		{
			--lastIte;
			undo_.pop_back();
		}
	}
	undo_.push_back(State());
	undoIte_ = undo_.end();
	--undoIte_;
	undoIte_->nbTrianglesState_ = triangles_.size();
	undoIte_->nbVerticesState_ = vertices_.size();
	undoIte_->aabbState_ = octree_->getAabbSplit();
}

/** Push verts and tris */
void Mesh::pushState(const std::vector<int> &iTris, const std::vector<int> &iVerts)
{
	TriangleVector &tState = undoIte_->tState_;
	VertexVector &vState = undoIte_->vState_;
	int nbTris = iTris.size();
	for(int i=0;i<nbTris;++i)
	{
		Triangle &t = triangles_[iTris[i]];
		if(t.stateFlag_!=Mesh::stateMask_)
		{
			t.stateFlag_ = Mesh::stateMask_;
			tState.push_back(t);
		}
	}
	int nbVerts = iVerts.size();
	for(int i=0;i<nbVerts;++i)
	{
		Vertex &v = vertices_[iVerts[i]];
		if(v.stateFlag_!=Mesh::stateMask_)
		{
			v.stateFlag_ = Mesh::stateMask_;
			vState.push_back(v);
		}
	}
}

/** Undo (also push_back the redo) */
void Mesh::undo()
{
	if(!undo_.size() || beginIte_) {
		return;
  }
	State redo;
	int nbTriangles = triangles_.size();
	int nbVertices = vertices_.size();
	redo.nbTrianglesState_ = nbTriangles;
	redo.nbVerticesState_ = nbVertices;
	redo.aabbState_ = octree_->getAabbSplit();
	TriangleVector &tRedoState = redo.tState_;
	VertexVector &vRedoState = redo.vState_;

	int nbTrianglesState  = undoIte_->nbTrianglesState_;
	int nbVerticesState  = undoIte_->nbVerticesState_;
	TriangleVector &tUndoState = undoIte_->tState_;
	VertexVector &vUndoState = undoIte_->vState_;

	int nbTris = tUndoState.size();
	int nbVerts = vUndoState.size();
	//REDO
	if(nbTrianglesState<nbTriangles)
	{
		for(int i=nbTrianglesState;i<nbTriangles;++i) tRedoState.push_back(triangles_[i]);
		for(int i=0;i<nbTris;++i)
		{
			Triangle &t = tUndoState[i];
			if(t.id_<nbTrianglesState) tRedoState.push_back(triangles_[t.id_]);
		}
		triangles_.resize(nbTrianglesState);
	}
	else
	{
		triangles_.resize(nbTrianglesState);
		for(int i=0;i<nbTris;++i)
		{
			Triangle &t = tUndoState[i];
			if(t.id_<nbTriangles) tRedoState.push_back(triangles_[t.id_]);
		}
	}
	if(nbVerticesState<nbVertices)
	{
		for(int i=nbVerticesState;i<nbVertices;++i) vRedoState.push_back(vertices_[i]);
		for(int i=0;i<nbVerts;++i)
		{
			Vertex &v = vUndoState[i];
			if(v.id_<nbVerticesState) vRedoState.push_back(vertices_[v.id_]);
		}
		vertices_.resize(nbVerticesState, Vertex(Vector3::Zero()));
	}
	else
	{
		vertices_.resize(nbVerticesState, Vertex(Vector3::Zero()));
		for(int i=0;i<nbVerts;++i)
		{
			Vertex &v = vUndoState[i];
			if(v.id_<nbVertices) vRedoState.push_back(vertices_[v.id_]);
		}
	}
	//UNDO
	for(int i=0;i<nbTris;++i)
	{
		Triangle &t = tUndoState[i];
		if(t.id_<nbTrianglesState) {
			triangles_[t.id_] = t;
    }
	}
	for(int i=0;i<nbVerts;++i)
	{
		Vertex &v = vUndoState[i];
		if(v.id_<nbVerticesState) {
			vertices_[v.id_] = v;
    }
	}
	recomputeOctree(undoIte_->aabbState_);
	initVBO();
	redo_.push_back(redo);
	if(undoIte_!=undo_.begin())
	{
		beginIte_ = false;
		--undoIte_;
	}
	else
  {
		beginIte_ = true;
  }
}

/** Redo */
void Mesh::redo()
{
	if(!redo_.size()) {
		return;
  }
	std::list<State>::iterator redoIte_ = redo_.end();
	--redoIte_;
	int nbTrianglesState  = redoIte_->nbTrianglesState_;
	int nbVerticesState  = redoIte_->nbVerticesState_;
	TriangleVector &tRedoState = redoIte_->tState_;
	VertexVector &vRedoState = redoIte_->vState_;
	triangles_.resize(nbTrianglesState);
	vertices_.resize(nbVerticesState, Vertex(Vector3::Zero()));
	int nbTris = tRedoState.size();
	for(int i=0;i<nbTris;++i)
	{
		Triangle &t = tRedoState[i];
		triangles_[t.id_] = t;
	}
	int nbVerts = vRedoState.size();
	for(int i=0;i<nbVerts;++i)
	{
		Vertex &v = vRedoState[i];
		vertices_[v.id_] = v;
	}
	recomputeOctree(redoIte_->aabbState_);
	initVBO();
	if(!beginIte_) {
		++undoIte_;
  } else {
		beginIte_ = false;
  }
	redo_.pop_back();
}

/** Recompute octree */
void Mesh::recomputeOctree(const Aabb &aabbSplit)
{
	int nbTriangles = triangles_.size();
	std::vector<int> triangles(nbTriangles);
#pragma omp parallel for
	for (int i=0;i<nbTriangles;++i) {
		triangles[i] = i;
  }
	++Triangle::tagMask_;
	delete octree_;
	octree_ = new Octree();
	octree_->build(this, triangles, aabbSplit);
}
