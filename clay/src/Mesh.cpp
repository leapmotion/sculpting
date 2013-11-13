#include "StdAfx.h"
#include "Mesh.h"
#include "Octree.h"
#include <iostream>

const float Mesh::globalScale_ = 500.f;
int Mesh::stateMask_ = 1;
const int undoLimit_ = 10;

/** Constructor */
Mesh::Mesh() : center_(Vector3::Zero()), scale_(1), lastUpdateTime_(0.0),
  octree_(0), rotationMatrix_(Matrix4x4::Identity()), beginIte_(false), verticesBuffer_(GL_ARRAY_BUFFER),
  normalsBuffer_(GL_ARRAY_BUFFER), indicesBuffer_(GL_ELEMENT_ARRAY_BUFFER), colorsBuffer_(GL_ARRAY_BUFFER),
  rotationOrigin_(Vector3::Zero()), rotationAxis_(Vector3::UnitY()), rotationVelocity_(0.0f), curRotation_(0.0f),
  verticesBufferCount_(0), indicesBufferCount_(0), reallocateVerticesBuffer_(true), reallocateIndicesBuffer_(true),
  undoPending_(false), redoPending_(false), nbGPUTriangles(0), pendingGPUTriangles(0), translation_(Vector3::Zero())
{ }

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
const Triangle& Mesh::getTriangle(int i) const { return triangles_[i]; }
Vertex& Mesh::getVertex(int i) { return vertices_[i]; }
const Vertex& Mesh::getVertex(int i) const { return vertices_[i]; }
int Mesh::getNbTriangles() const { return triangles_.size(); }
int Mesh::getNbVertices() const { return vertices_.size(); }
Vector3 Mesh::getCenter() const { return center_; }
Octree* Mesh::getOctree() const { return octree_; }
float Mesh::getScale() const { return scale_; }
Matrix4x4 Mesh::getTransformation() const {
  Matrix4x4 transMat = Tools::translationMatrix(translation_);
  return transMat * rotationMatrix_;
}
Matrix4x4 Mesh::getTransformation(double curTime) const {
  const float deltaTime = static_cast<float>(curTime - lastUpdateTime_);
  Matrix4x4 transMat = Tools::translationMatrix(translation_);
  Matrix4x4 rotMat = Tools::rotationMatrix(rotationAxis_, curRotation_ + deltaTime*getRotationVelocity());
  return transMat * rotMat;
}
void Mesh::setRotationVelocity(float vel) { rotationVelocity_ = vel; }
void Mesh::updateRotation(double curTime) {
  rotationVelocitySmoother_.Update(rotationVelocity_, curTime, 0.95f);
  const float deltaTime = static_cast<float>(curTime - lastUpdateTime_);
  curRotation_ += deltaTime*getRotationVelocity();
  rotationMatrix_ = Tools::rotationMatrix(rotationAxis_, curRotation_);
  lastUpdateTime_ = curTime;
}
const Vector3& Mesh::getRotationOrigin() const { return rotationOrigin_; }
const Vector3& Mesh::getRotationAxis() const { return rotationAxis_; }
float Mesh::getRotationVelocity() const { return rotationVelocitySmoother_.value; }

/** Return all the triangles linked to a group of vertices */
std::vector<int> Mesh::getTrianglesFromVertices(const std::vector<int> &iVerts)
{
  ++Triangle::tagMask_;
  std::vector<int> triangles;
  const int nbVerts = iVerts.size();
  for(int i=0;i<nbVerts;++i)
  {
    const std::vector<int> &iTris = vertices_[iVerts[i]].tIndices_;
    int nbTris = iTris.size();
    for(int j=0;j<nbTris;++j)
    {
      const int iTri = iTris[j];
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
  const int nbTris = iTris.size();
  for(int i=0;i<nbTris;++i)
  {
    const Triangle &t=triangles_[iTris[i]];
    for (int j=0; j<3; j++) {
      const int iVer = t.vIndices_[j];
      if (vertices_[iVer].tagFlag_ != Vertex::tagMask_) {
        vertices.push_back(iVer);
        vertices_[iVer].tagFlag_ = Vertex::tagMask_;
      }
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
  translation_ = destination - center_;
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
  glDrawElements(GL_TRIANGLES, nbGPUTriangles*3, GL_UNSIGNED_INT, 0);
  indicesBuffer_.release();
  glDisableVertexAttribArray(vertex);
  glDisableVertexAttribArray(normal);
  glDisableVertexAttribArray(color);

  normalsBuffer_.release();
  verticesBuffer_.release();
  colorsBuffer_.release();
}

void Mesh::drawVerticesOnly(GLint vertex) {
  verticesBuffer_.bind();
  glEnableVertexAttribArray(vertex);
  GLBuffer::checkError();
  glVertexAttribPointer(vertex, 3, GL_FLOAT, GL_TRUE, 0, 0);
  GLBuffer::checkError();

  indicesBuffer_.bind();
  glDrawElements(GL_TRIANGLES, nbGPUTriangles*3, GL_UNSIGNED_INT, 0);
  indicesBuffer_.release();
  glDisableVertexAttribArray(vertex);

  verticesBuffer_.release();
}

void Mesh::drawOctree() const {
  glPushMatrix();
  glMultMatrixf(getTransformation().data());
  octree_->draw();
  glPopMatrix();
}

void Mesh::initVertexVBO() {
  const int nbVertices = getNbVertices();
  verticesBufferCount_ = 2*nbVertices;
  const int verticesBytes = verticesBufferCount_*3*sizeof(GLfloat);

  if (verticesBuffer_.isCreated()) {
    verticesBuffer_.destroy();
  }
  verticesBuffer_.create();
  verticesBuffer_.bind();
  verticesBuffer_.allocate(0, verticesBytes, GL_DYNAMIC_DRAW);
  verticesBuffer_.release();

  if (normalsBuffer_.isCreated()) {
    normalsBuffer_.destroy();
  }
  normalsBuffer_.create();
  normalsBuffer_.bind();
  normalsBuffer_.allocate(0, verticesBytes, GL_DYNAMIC_DRAW);
  normalsBuffer_.release();

  if (colorsBuffer_.isCreated()) {
    colorsBuffer_.destroy();
  }
  colorsBuffer_.create();
  colorsBuffer_.bind();
  colorsBuffer_.allocate(0, verticesBytes, GL_DYNAMIC_DRAW);
  colorsBuffer_.release();

  reallocateVerticesBuffer_ = false;
}

void Mesh::initIndexVBO() {
  const int nbTriangles = getNbTriangles();
  indicesBufferCount_ = 2*nbTriangles;
  const int indicesBytes = indicesBufferCount_*3*sizeof(GLuint);

  if (indicesBuffer_.isCreated()) {
    indicesBuffer_.destroy();
  }
  indicesBuffer_.create();
  indicesBuffer_.bind();
  indicesBuffer_.allocate(0, indicesBytes, GL_DYNAMIC_DRAW);
  indicesBuffer_.release();

  reallocateIndicesBuffer_ = false;
}

void Mesh::reinitVerticesBuffer() {
  vertexUpdates_.clear();
  reallocateVerticesBuffer_ = true;

  const int nbVertices = getNbVertices();
  for (int i=0; i<nbVertices; i++) {
    VertexUpdate update;
    update.idx = i;
    update.color = vertices_[i].material_;
    update.normal = vertices_[i].normal_;
    update.pos = vertices_[i];
    vertexUpdates_.push_back(update);
  }
}

void Mesh::reinitIndicesBuffer() {
  indexUpdates_.clear();
  reallocateIndicesBuffer_ = true;

  const int nbTriangles = getNbTriangles();
  for (int i=0; i<nbTriangles; i++) {
    IndexUpdate update;
    update.idx = i;
    update.indices[0] = triangles_[i].vIndices_[0];
    update.indices[1] = triangles_[i].vIndices_[1];
    update.indices[2] = triangles_[i].vIndices_[2];
    indexUpdates_.push_back(update);
  }
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
  //matTransform_ = Tools::scaleMatrix(scale_)*matTransform_;
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

  reinitIndicesBuffer();
  reinitVerticesBuffer();
  pendingGPUTriangles = getNbTriangles();
}

/** Update geometry  */
void Mesh::updateMesh(const std::vector<int> &iTris, const std::vector<int> &iVerts)
{
  const int nbTris=iTris.size();
  const int nbVerts=iVerts.size();
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
#if _WIN32
  assert(_CrtCheckMemory());
#endif

  std::unique_lock<std::mutex> lock(bufferMutex_);

  if (getNbTriangles() < indicesBufferCount_) {
    // within storage bounds, so it's OK to only update part of the buffer
    for (int i=0; i<nbTris; i++) {
      IndexUpdate update;
      update.idx = iTris[i];
      update.indices[0] = triangles_[update.idx].vIndices_[0];
      update.indices[1] = triangles_[update.idx].vIndices_[1];
      update.indices[2] = triangles_[update.idx].vIndices_[2];
      indexUpdates_.push_back(update);
    }
  } else {
    // not enough space, reallocate
    reinitIndicesBuffer();
  }

  if (getNbVertices() < verticesBufferCount_) {
    // within storage bounds, so it's OK to only update part of the buffer
    for (int i=0; i<nbVerts; i++) {
      VertexUpdate update;
      update.idx = iVerts[i];
      update.color = vertices_[update.idx].material_;
      update.normal = vertices_[update.idx].normal_;
      update.pos = vertices_[update.idx];
      vertexUpdates_.push_back(update);
    }
  } else {
    // not enough space, reallocate
    reinitVerticesBuffer();
  }

  pendingGPUTriangles = getNbTriangles();
}

void Mesh::updateGPUBuffers() {
  std::unique_lock<std::mutex> lock(bufferMutex_);

  if (reallocateIndicesBuffer_) {
    initIndexVBO();
  }

  GLuint* indicesArray;
  indicesBuffer_.bind(); indicesArray = (GLuint*)indicesBuffer_.map(GL_WRITE_ONLY); indicesBuffer_.release();
  const int nbTris = indexUpdates_.size();
  for (int i=0; i<nbTris; i++) {
    const IndexUpdate& cur = indexUpdates_[i];
    const int j = cur.idx;
    indicesArray[j*3] = cur.indices[0];
    indicesArray[j*3+1] = cur.indices[1];
    indicesArray[j*3+2] = cur.indices[2];
  }
  indexUpdates_.clear();
  indicesBuffer_.bind(); indicesBuffer_.unmap(); indicesBuffer_.release();

  //int numVertex = 0;
  if (reallocateVerticesBuffer_) {
    initVertexVBO();
  }

  GLfloat* verticesArray;
  GLfloat* normalsArray;
  GLfloat* colorsArray;
  verticesBuffer_.bind(); verticesArray = (GLfloat*)verticesBuffer_.map(GL_WRITE_ONLY); verticesBuffer_.release();
  normalsBuffer_.bind(); normalsArray = (GLfloat*)normalsBuffer_.map(GL_WRITE_ONLY); normalsBuffer_.release();
  colorsBuffer_.bind(); colorsArray = (GLfloat*)colorsBuffer_.map(GL_WRITE_ONLY); colorsBuffer_.release();

  const int nbVerts = vertexUpdates_.size();
  for (int i=0; i<nbVerts; i++) {
    const VertexUpdate& cur = vertexUpdates_[i];
    const int j = cur.idx;
    verticesArray[j*3] = cur.pos.x();
    verticesArray[j*3+1] = cur.pos.y();
    verticesArray[j*3+2] = cur.pos.z();
    normalsArray[j*3] = cur.normal.x();
    normalsArray[j*3+1] = cur.normal.y();
    normalsArray[j*3+2] = cur.normal.z();
    colorsArray[j*3] = cur.color.x();
    colorsArray[j*3+1] = cur.color.y();
    colorsArray[j*3+2] = cur.color.z();
  }
  vertexUpdates_.clear();
  verticesBuffer_.bind(); verticesBuffer_.unmap(); verticesBuffer_.release();
  normalsBuffer_.bind(); normalsBuffer_.unmap(); normalsBuffer_.release();
  colorsBuffer_.bind(); colorsBuffer_.unmap(); colorsBuffer_.release();

  nbGPUTriangles = pendingGPUTriangles;
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
  for (int i=0;i<nbVers;++i)
  {
    Vertex &vert=vertices_[iVerts[i]];
    const std::vector<int> &iTri = vert.tIndices_;
    int nbTri = iTri.size();
    Vector3 normal(Vector3::Zero());
    for (int j=0;j<nbTri;++j)
      normal+=triangles_[iTri[j]].normal_;
    float length = normal.norm();
    if (nbTri != 0) {
      if (length < 0.0001f && nbTri > 0) {
        // normals added up to zero length, so just pick one
        normal = triangles_[iTri[0]].normal_;
      } else {
        normal = normal/length;
      }
      vert.normal_ = normal;
      assert(fabs(vert.normal_.squaredNorm() - 1.0f) < 0.001f);
    }
  }
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

void Mesh::undo() {
  undoPending_ = true;
}

void Mesh::redo() {
  redoPending_ = true;
}

void Mesh::handleUndoRedo() {
  if (undoPending_) {
    performUndo();
  }
  if (redoPending_) {
    performRedo();
  }
}

/** Undo (also push_back the redo) */
void Mesh::performUndo()
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
  reinitVerticesBuffer();
  reinitIndicesBuffer();
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
  undoPending_ = false;
}

/** Redo */
void Mesh::performRedo()
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
  reinitVerticesBuffer();
  reinitIndicesBuffer();
  if(!beginIte_) {
    ++undoIte_;
  } else {
    beginIte_ = false;
  }
  redo_.pop_back();
  redoPending_ = false;
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
